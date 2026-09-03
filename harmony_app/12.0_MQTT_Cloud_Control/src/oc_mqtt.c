
#include <string.h>
#include <stdio.h>
#include "MQTTClient.h"
#include <unistd.h>
#include "cJSON.h"
#include "cmsis_os2.h"
#include <oc_mqtt.h>
#include <oc_mqtt_profile_package.h>

typedef struct
{
    char                        *device_id;
    fn_oc_mqtt_profile_rcvdeal   rcvfunc;
}oc_mqtt_profile_cb_t;

static oc_mqtt_profile_cb_t s_oc_mqtt_profile_cb;
static char init_ok = FALSE;
static MQTTClient mq_client;
struct bp_oc_info oc_info;
struct oc_device
{
    struct bp_oc_info *oc_info;

    void(*cmd_rsp_cb)(uint8_t *recv_data, size_t recv_size, uint8_t **resp_data, size_t *resp_size);

} oc_mqtt;

/* 命令响应异步化：request_id 队列 + 独立回响应线程。
   绝不能在 mqtt_callback(MQTT 接收线程)里同步 publish 响应：
   MQTTRun 持有 paho 的二值信号量(osSemaphoreNew(1,1)，不可重入)调 cycle()→本回调，
   回调内再 MQTTPublish 会对同一信号量二次 acquire → 自死锁，响应发不出、keepalive 停摆
   (云端 IOTDA.014111 命令超时 + 设备掉线)。改由 oc_resp_worker 独立线程发布。 */
#define OC_RESP_Q_LEN  8
#define OC_RID_MAX     80
typedef struct { char rid[OC_RID_MAX]; } oc_resp_item_t;
static osMessageQueueId_t s_resp_q = NULL;

static void oc_resp_worker(void *arg)
{
    (void)arg;
    oc_resp_item_t item;

    for (;;)
    {
        if (osMessageQueueGet(s_resp_q, &item, NULL, osWaitForever) != osOK)
        {
            continue;
        }
        if (item.rid[0] == '\0')
        {
            continue;
        }
        oc_mqtt_profile_cmdresp_t cmdresp;
        cmdresp.ret_code = 0;
        cmdresp.ret_name = NULL;
        cmdresp.request_id = item.rid;
        cmdresp.paras = NULL;
        int rr = oc_mqtt_profile_cmdresp(CLOUD_DEVICE_ID, &cmdresp);
        printf("[OC] cmdresp rid=%s rc=%d\r\n", item.rid, rr);
    }
}

 void mqtt_callback(MessageData *msg_data)
{
    char topic[160];
    int tlen;
    char *rid;

    LOS_ASSERT(msg_data);

    /* 命令 topic 非 null 结尾，先拷成 C 串：
       $oc/devices/{id}/sys/commands/request_id={rid} */
    tlen = msg_data->topicName->lenstring.len;
    if (tlen <= 0)
    {
        return;
    }
    if (tlen >= (int)sizeof(topic))
    {
        tlen = (int)sizeof(topic) - 1;
    }
    memcpy(topic, msg_data->topicName->lenstring.data, tlen);
    topic[tlen] = '\0';

    /* 交应用回调处理命令（解析 + 入队运动）*/
    if (oc_mqtt.cmd_rsp_cb != NULL)
    {
        oc_mqtt.cmd_rsp_cb((uint8_t *) msg_data->message->payload,
                           msg_data->message->payloadlen, NULL, NULL);
    }

    /* 提取 request_id 投递到响应队列，由 oc_resp_worker 异步回 {"result_code":0} 到
       $oc/devices/{id}/sys/commands/response/request_id={rid}。本回调只入队、不 publish。 */
    rid = strstr(topic, "request_id=");
    if (rid != NULL && s_resp_q != NULL)
    {
        oc_resp_item_t item;

        rid += strlen("request_id=");
        memset(&item, 0, sizeof(item));
        strncpy(item.rid, rid, sizeof(item.rid) - 1);
        if (osMessageQueuePut(s_resp_q, &item, 0U, 0U) != osOK)
        {
            printf("[OC] resp queue full, drop rid=%s\r\n", item.rid);
        }
    }
}
unsigned char *oc_mqtt_buf;
unsigned char *oc_mqtt_readbuf;
int buf_size;

Network n;
MQTTPacket_connectData data = MQTTPacket_connectData_initializer;  

static int oc_mqtt_entry(void)
{
    int rc = 0;
    
	NetworkInit(&n);
	int net_rc = NetworkConnect(&n, OC_SERVER_IP, OC_SERVER_PORT);
	printf("[OC] NetworkConnect(%s:%d) rc=%d\r\n", OC_SERVER_IP, OC_SERVER_PORT, net_rc);
	if (net_rc != 0)
	{
		printf("[OC] network connect fail, check host/wifi\r\n");
		return -2;
	}

    buf_size  = 2048;
    oc_mqtt_buf = (unsigned char *) malloc(buf_size);
    oc_mqtt_readbuf = (unsigned char *) malloc(buf_size);
    if (!(oc_mqtt_buf && oc_mqtt_readbuf))
    {
        printf("No memory for MQTT client buffer!");
        return -2;
    }

	MQTTClientInit(&mq_client, &n, 1000, oc_mqtt_buf, buf_size, oc_mqtt_readbuf, buf_size);
	
    MQTTStartTask(&mq_client);


    data.keepAliveInterval = 30;
    data.cleansession = 1;
	data.clientID.cstring = oc_info.client_id;
	data.username.cstring = oc_info.username;
	data.password.cstring = oc_info.password;
	data.MQTTVersion =3;

	
    mq_client.defaultMessageHandler = mqtt_callback;

	rc = MQTTConnect(&mq_client, &data);
	printf("[OC] MQTTConnect rc=%d\r\n", rc);

	/* 连接成功后订阅命令下发主题，云端 move 命令经 mqtt_callback → cmd_rsp_cb 到达应用 */
	if (rc == SUCCESS)
	{
		char cmd_topic[128];
		snprintf(cmd_topic, sizeof(cmd_topic), "$oc/devices/%s/sys/commands/#", CLOUD_DEVICE_ID);
		int sub_rc = MQTTSubscribe(&mq_client, cmd_topic, QOS1, mqtt_callback);
		printf("[OC] MQTTSubscribe %s rc=%d\r\n", cmd_topic, sub_rc);
	}

    return rc;
    
}


void device_info_init(char *client_id, char * username, char *password)
{
    oc_info.user_device_id_flg = 1;
    strncpy(oc_info.client_id,client_id, strlen(client_id));
    strncpy(oc_info.username,username, strlen(username));
    strncpy(oc_info.password,password, strlen(password));

}

/**
 * oc mqtt client init.
 *
 * @param   NULL
 *
 * @return  0 : init success
 *         -1 : get device info fail
 *         -2 : oc mqtt client init fail
 */
int oc_mqtt_init(void)
{
    int result = 0;

    if (init_ok)
    {
        //LOG_D("oc mqtt already init!");
        return 0;
    }

    /* 创建命令响应队列 + 独立回响应线程（仅一次）*/
    if (s_resp_q == NULL)
    {
        s_resp_q = osMessageQueueNew(OC_RESP_Q_LEN, sizeof(oc_resp_item_t), NULL);
    }
    if (s_resp_q != NULL)
    {
        osThreadAttr_t resp_attr;
        memset(&resp_attr, 0, sizeof(resp_attr));
        resp_attr.name = "oc_resp";
        resp_attr.stack_size = 4096;
        resp_attr.priority = osPriorityNormal;
        (void)osThreadNew(oc_resp_worker, NULL, &resp_attr);
    }

    if (oc_mqtt_entry() < 0)
    {
        result = -2;
        goto __exit;
    }
    __exit:
    if (!result)
    {
        //LOG_I("oc package(V%s) initialize success.", oc_SW_VERSION);
        init_ok = 0;
    }
    else
    {
        //LOG_E("oc package(V%s) initialize failed(%d).", oc_SW_VERSION, result);
    }
    return result;
}
/**
 * set the command responses call back function
 *
 * @param   cmd_rsp_cb  command responses call back function
 *
 * @return  0 : set success
 *         -1 : function is null
 */
void oc_set_cmd_rsp_cb(void (*cmd_rsp_cb)(uint8_t *recv_data, uint32_t recv_size, uint8_t **resp_data, uint32_t *resp_size))
{

    oc_mqtt.cmd_rsp_cb = cmd_rsp_cb;

}


/**
 * mqtt publish msg to topic
 *
 * @param   topic   target topic
 * @param   msg     message to be sent
 * @param   len     message length
 *
 * @return  0 : publish success
 *         -1 : publish fail
 */
 int oc_mqtt_publish(char  *topic,uint8_t *msg,int msg_len,int qos)
{
    MQTTMessage message;

    LOS_ASSERT(topic);
    LOS_ASSERT(msg);

    message.qos = qos;
    message.retained = 0;
    message.payload = (void *) msg;
    message.payloadlen = msg_len;

    if (MQTTPublish(&mq_client, topic, &message) < 0)
    {
        return -1;
    }

    return 0;
}
///< use this function to make a topic to publish
///< if request_id  is needed depends on the fmt
static char *topic_make(char *fmt, char *device_id, char *request_id)
{
    int len;
    char *ret = NULL;

    if(NULL == device_id)
    {
        return ret;
    }
    len = strlen(fmt) + strlen(device_id);
    if(NULL != request_id)
    {
        len += strlen(request_id);
    }

    ret = malloc(len);
    if(NULL != ret)
    {
        (void) snprintf(ret,len,fmt,device_id,request_id);
    }
    return ret;
}


///< use this function to report the messsage
#define CN_OC_MQTT_PROFILE_MSGUP_TOPICFMT   "$oc/devices/%s/sys/messages/up"
int oc_mqtt_profile_msgup(char *deviceid,oc_mqtt_profile_msgup_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL == payload) || (NULL == payload->msg))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_MSGUP_TOPICFMT, deviceid,NULL);
    msg = oc_mqtt_profile_package_msgup(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}

#define CN_OC_MQTT_PROFILE_PROPERTYREPORT_TOPICFMT   "$oc/devices/%s/sys/properties/report"
int oc_mqtt_profile_propertyreport(char *deviceid,oc_mqtt_profile_service_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL== payload) || (NULL== payload->service_id) || (NULL == payload->service_property))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_PROPERTYREPORT_TOPICFMT, deviceid,NULL);
    msg = oc_mqtt_profile_package_propertyreport(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
            printf("上报数据成功！！！");
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}

#define CN_OC_MQTT_PROFILE_GWPROPERTYREPORT_TOPICFMT   "$oc/devices/%s/sys/gateway/sub_devices/properties/report"
int oc_mqtt_profile_gwpropertyreport(char *deviceid,oc_mqtt_profile_device_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL== payload) || (NULL == payload->subdevice_id)||(NULL== payload->subdevice_property) ||\
       (NULL== payload->subdevice_property->service_id)||(NULL== payload->subdevice_property->service_property))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_GWPROPERTYREPORT_TOPICFMT, deviceid,NULL);
    msg = oc_mqtt_profile_package_gwpropertyreport(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}


#define CN_OC_MQTT_PROFILE_ROPERTYSETRESP_TOPICFMT   "$oc/devices/%s/sys/properties/set/response/request_id=%s"
int oc_mqtt_profile_propertysetresp(char *deviceid,oc_mqtt_profile_propertysetresp_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL == payload) || (NULL == payload->request_id))
    {
        return ret;
    }
    topic = topic_make(CN_OC_MQTT_PROFILE_ROPERTYSETRESP_TOPICFMT, deviceid,payload->request_id);
    msg = oc_mqtt_profile_package_propertysetresp(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}


#define CN_OC_MQTT_PROFILE_ROPERTYGETRESP_TOPICFMT   "$oc/devices/%s/sys/properties/get/response/request_id=%s"
int oc_mqtt_profile_propertygetresp(char *deviceid,oc_mqtt_profile_propertygetresp_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL== payload) || (NULL == payload->request_id) || \
       (NULL== payload->services->service_id) || (NULL == payload->services->service_property))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_ROPERTYGETRESP_TOPICFMT, deviceid,payload->request_id);
    msg = oc_mqtt_profile_package_propertygetresp(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}

#define CN_OC_MQTT_PROFILE_CMDRESP_TOPICFMT   "$oc/devices/%s/sys/commands/response/request_id=%s"
int oc_mqtt_profile_cmdresp(char *deviceid,oc_mqtt_profile_cmdresp_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL == payload) || (NULL == payload->request_id))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_CMDRESP_TOPICFMT, deviceid,payload->request_id);
    msg = oc_mqtt_profile_package_cmdresp(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}
