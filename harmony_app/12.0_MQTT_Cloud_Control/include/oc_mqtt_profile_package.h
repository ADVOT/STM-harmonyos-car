
#ifndef LITEOS_LAB_IOT_LINK_OC_OC_MQTT_OC_MQTT_PROFILE_OC_MQTT_PROFILE_PACKAGE_H_
#define LITEOS_LAB_IOT_LINK_OC_OC_MQTT_OC_MQTT_PROFILE_OC_MQTT_PROFILE_PACKAGE_H_



///< defines for the package tools
char *oc_mqtt_profile_package_msgup(oc_mqtt_profile_msgup_t *payload);
char *oc_mqtt_profile_package_propertyreport(oc_mqtt_profile_service_t *payload);
char *oc_mqtt_profile_package_gwpropertyreport(oc_mqtt_profile_device_t *payload);
char *oc_mqtt_profile_package_propertysetresp(oc_mqtt_profile_propertysetresp_t *payload);
char *oc_mqtt_profile_package_propertygetresp(oc_mqtt_profile_propertygetresp_t *payload);
char *oc_mqtt_profile_package_cmdresp(oc_mqtt_profile_cmdresp_t *payload);





#endif /* LITEOS_LAB_IOT_LINK_OC_OC_MQTT_OC_MQTT_PROFILE_OC_MQTT_PROFILE_PACKAGE_H_ */
