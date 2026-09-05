
#ifndef LITEOS_LAB_IOT_LINK_OC_OC_MQTT_OC_MQTT_PROFILE_OC_MQTT_PROFILE_H_
#define LITEOS_LAB_IOT_LINK_OC_OC_MQTT_OC_MQTT_PROFILE_OC_MQTT_PROFILE_H_

#include <stdint.h>
#include "cloud_config.h"

/* 接入域名/端口统一从 cloud_config.h 取（USER填自己实例的值） */
#define OC_SERVER_IP                CLOUD_MQTT_HOST
#define OC_SERVER_PORT              CLOUD_MQTT_PORT
#define OC_CLIENT_ID_LEN          128
#define OC_USERNAME_LEN         128
#define OC_PASSWORD_LEN          128


///< UP means the device send data to the cloud
typedef enum
{
    EN_OC_MQTT_PROFILE_MSG_TYPE_UP_MSGUP = 0,            ///< DEVICE REPORT MESSAG DIRECTLY
    EN_OC_MQTT_PROFILE_MSG_TYPE_UP_PROPERTYREPORT,       ///< DEVICE REPORT PROPERTY
    EN_OC_MQTT_PROFILE_MSG_TYPE_UP_SUBPROPERTYREPORT,    ///< GATWAY REPORT SUBDEVICEPROPERTY
    EN_OC_MQTT_PROFILE_MSG_TYPE_UP_PROPERTYSETRESPONSE,  ///< RESPONSE TO THE PROPERTY SET
    EN_OC_MQTT_PROFILE_MSG_TYPE_UP_PROPERTYGETRESPONSE,  ///< RESPONSE TO THE PROPERTY GET
    EN_OC_MQTT_PROFILE_MSG_TYPE_UP_CMDRESPONSE,          ///< RESPONSE TO THE COMMAND
    EN_OC_MQTT_PROFILE_MSG_TYPE_UP_LAST,
}en_oc_mqtt_profile_msg_type_up_t;


///< DOWN means the cloud send data to the device
typedef enum
{
    EN_OC_MQTT_PROFILE_MSG_TYPE_DOWN_MSGDOWN = 0,        ///< THIS IS THE MESSAGE DOWN
    EN_OC_MQTT_PROFILE_MSG_TYPE_DOWN_COMMANDS,           ///< THIS IS THE PLATFORM COMMANDS
    EN_OC_MQTT_PROFILE_MSG_TYPE_DOWN_PROPERTYSET,        ///< THIS IS THE PLATFORM SET PROPERTY
    EN_OC_MQTT_PROFILE_MSG_TYPE_DOWN_PROPERTYGET,        ///< THIS IS THE PLATFORM GET PROPERTY
    EN_OC_MQTT_PROFILE_MSG_TYPE_DOWN_EVENT,              ///< THIS IS THE EVENT
    EN_OC_MQTT_PROFILE_MSG_TYPE_DOWN_LAST,
}en_oc_mqtt_profile_msg_type_down_t;


////< enum all the data type for the oc profile
typedef enum
{
    EN_OC_MQTT_PROFILE_VALUE_INT = 0,
    EN_OC_MQTT_PROFILE_VALUE_LONG,
    EN_OC_MQTT_PROFILE_VALUE_FLOAT,
    EN_OC_MQTT_PROFILE_VALUE_STRING,           ///< must be ended with '\0'
    EN_OC_MQTT_PROFILE_VALUE_LAST,
}en_oc_profile_data_t;



typedef struct
{
    void                 *nxt;   ///< ponit to the next key
    char                 *key;
    en_oc_profile_data_t  type;
    void                 *value;
}oc_mqtt_profile_kv_t;


typedef struct
{
    en_oc_mqtt_profile_msg_type_down_t type;       ///< defined as en_oc_mqtt_profile_msg_type_down_t
    char *request_id;                              ///< get from the topic, NULL if not supplied
    void *msg;                                     ///< the send from the cloud platform
    int   msg_len;                                 ///< the message length
}oc_mqtt_profile_msgrcv_t;

/**
 * @brief: this the message from the cloud call back function
 *
 * @param[in] payload:defined as oc_mqtt_profile_msgrcv_t
 *
 * @return: not used yet
 *
 * */
typedef int (*fn_oc_mqtt_profile_rcvdeal)(oc_mqtt_profile_msgrcv_t *payload);


typedef struct
{
    char *device_id;   ///< you could specify it,if NULL,then use the device_id which used to connect
    char *name;        ///< message name, could be NULL
    char *id;          ///< message id, could be NULL
    void *msg;         ///< message to send
    int   msg_len;     ///< message length
}oc_mqtt_profile_msgup_t;

/** @brief enum all the qos supported for the application */
typedef enum
{
    en_mqtt_al_qos_0 = 0,     ///< mqtt QOS 0
    en_mqtt_al_qos_1,         ///< mqtt QOS 1
    en_mqtt_al_qos_2,         ///< mqtt QOS 2
    en_mqtt_al_qos_err
}en_mqtt_al_qos_t;

typedef enum
{
    en_oc_mqtt_err_ok          = 0,      ///< this means the status ok
    en_oc_mqtt_err_parafmt,              ///< this means the parameter err format
    en_oc_mqtt_err_network,              ///< this means the network wrong status
    en_oc_mqtt_err_conversion,           ///< this means the mqtt version err
    en_oc_mqtt_err_conclientid,          ///< this means the client id is err
    en_oc_mqtt_err_conserver,            ///< this means the server refused the service for some reason(likely the id and pwd)
    en_oc_mqtt_err_conuserpwd,           ///< bad user name or pwd
    en_oc_mqtt_err_conclient,            ///< the client id /user/pwd is right, but does not allowed
    en_oc_mqtt_err_subscribe,            ///< this means subscribe the topic failed
    en_oc_mqtt_err_unsubscribe,          ///< this means un-subscribe failed
    en_oc_mqtt_err_publish,              ///< this means publish the topic failed
    en_oc_mqtt_err_configured,           ///< this means we has configured, please deconfigured it and then do configure again
    en_oc_mqtt_err_noconfigured,         ///< this means we have not configure it yet,so could not connect
    en_oc_mqtt_err_noconected,           ///< this means the connection has not been built, so you could not send data
    en_oc_mqtt_err_gethubaddrtimeout,    ///< this means get the hub address timeout
    en_oc_mqtt_err_sysmem,               ///< this means the system memory is not enough
    en_oc_mqtt_err_system,               ///< this means that the system porting may have some problem,maybe not install yet
    en_oc_mqtt_err_last,
}en_oc_mqtt_err_code_t;


struct bp_oc_info
{
    char client_id[OC_CLIENT_ID_LEN];
    char username[OC_USERNAME_LEN];

    char password[OC_PASSWORD_LEN];


    char user_device_id_flg;
};
typedef struct bp_oc_info *bp_oc_info_t;

int oc_mqtt_init(void);

void device_info_init(char *client_id, char * username, char *password);

void oc_set_cmd_rsp_cb(void (*cmd_rsp_cb)(uint8_t *recv_data, uint32_t recv_size, uint8_t **resp_data, uint32_t *resp_size));

int oc_mqtt_publish(char  *topic,uint8_t *msg,int msg_len,int qos);
/**
 * @brief: use this function to send data to the platform without profile decode
 *
 * @param[in] deviceid: the cloud message receiver, if NULL then send to the connected one
 *
 * @param[in] payload: defined as oc_mqtt_profile_msgup_t
 *
 * @return :defined as en_oc_mqtt_err_code_t
 *
 * */
int oc_mqtt_profile_msgup(char *deviceid,oc_mqtt_profile_msgup_t *payload);


typedef struct
{
   void *nxt;
   char *service_id;                         ///< the service id in the profile, which could not be NULL
   char *event_time;                         ///< eventtime, which could be NULL means use the platform time
   oc_mqtt_profile_kv_t *service_property;   ///< the property in the profile, which could not be NULL
}oc_mqtt_profile_service_t;

/**
 * @brief: use this function to send data to the platform with profile decode
 *
 * @param[in] deviceid: the cloud message receiver, if NULL then send to the connected one
 *
 * @param[in] payload: properties list to send to the platform
 *
 * @return :defined as en_oc_mqtt_err_code_t
 *
 * */
int oc_mqtt_profile_propertyreport(char *deviceid,oc_mqtt_profile_service_t *payload);

typedef struct
{
    void *nxt;                                                  ///< maybe much more
    char                                *subdevice_id;          ///< the specified device, which could not be NULL
    oc_mqtt_profile_service_t           *subdevice_property;    ///< the property of the specified device, which could not be NULL
}oc_mqtt_profile_device_t;

/**
 * @brief: use this function to send data to the platform with profile decode
 *
 * @param[in] deviceid: the cloud message receiver, if NULL then send to the connected one
 *
 * @param[in] payload:defined as oc_mqtt_profile_device_t
 *
 * @return :defined as en_oc_mqtt_err_code_t
 *
 * */
int oc_mqtt_profile_gwpropertyreport(char *deviceid,oc_mqtt_profile_device_t *payload);


typedef struct
{
    int     ret_code;           ///< response code, 0 success while others failed
    char   *ret_description;    ///< response description,maybe used when failed
    char   *request_id;         ///< specified by the setproperty command
}oc_mqtt_profile_propertysetresp_t;
/**
 * @brief: use this function to send the response to the setproperty command
 *
 * @param[in] deviceid: the cloud message receiver, if NULL then send to the connected one
 *
 * @param[in] payload: description as oc_mqtt_profile_propertysetresp_t
 *
 * @return :defined as en_oc_mqtt_err_code_t
 *
 * */
int oc_mqtt_profile_propertysetresp(char *deviceid,oc_mqtt_profile_propertysetresp_t *payload);


typedef struct
{
    char *request_id;                              ///< specified by the getproperty command
    oc_mqtt_profile_service_t  *services;          ///< defined as oc_mqtt_profile_service_t
}oc_mqtt_profile_propertygetresp_t;

/**
 * @brief: use this function to send the response to the setproperty command

 * @param[in] deviceid: the cloud message receiver, if NULL then send to the connected one

 * @param[in] payload: description as oc_mqtt_profile_propertygetresp_t
 *
 * @return :defined as en_oc_mqtt_err_code_t
 *
 * */
int oc_mqtt_profile_propertygetresp(char *deviceid,oc_mqtt_profile_propertygetresp_t *payload);


typedef struct
{
    int     ret_code;           ///< response code, 0 success while others failed
    char   *ret_name;           ///< response description,maybe used when failed
    char   *request_id;         ///< specified by the message command
    oc_mqtt_profile_kv_t  *paras;///< the command paras

}oc_mqtt_profile_cmdresp_t;
/**
 * @brief: use this function to send the response to the setproperty command
 *
 * @param[in] deviceid: the cloud message receiver, if NULL then send to the connected one
 *
 * @param[in] payload: description as oc_mqtt_profile_cmdresp_t
 *
 * @return :defined as en_oc_mqtt_err_code_t
 *
 * */
int oc_mqtt_profile_cmdresp(char *deviceid,oc_mqtt_profile_cmdresp_t *payload);





#endif /* LITEOS_LAB_IOT_LINK_OC_OC_MQTT_OC_MQTT_PROFILE_OC_MQTT_PROFILE_H_ */
