/*
  Project:			realtime_co2
  Description:	contains private configuration data not stored in github
*/

// Configuration Step 1: site environment variables
// set site altitude, in meters, to calibrate the SCD40
// const int SITE_ALTITUDE = 90; // Mercer Island, WA
const int SITE_ALTITUDE = 236; // Pasadena, CA (SuperCon!)

// Configuration Step 2: WiFi credentials
// set WiFi SSID and password if data endpoints are used
#if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT) || defined(DWEET)
	#define WIFI_SSID       "KleiOT"
	#define WIFI_PASS       "hJRggL467J9w"
	// #define WIFI_SSID       "Fascism sux"
	// #define WIFI_SSID       "katana"
	// #define WIFI_PASS       "redshirt"
	// #define WIFI_SSID       "snokingice"
	// #define WIFI_PASS       "snokingice"
#endif

// Configuration Step 3: MQTT credentials
// set MQTT broker login parameters
#ifdef MQTT
	// #define MQTT_BROKER		"192.168.1.27"
	// #define MQTT_BROKER		"91.121.93.94" // test.mosquitto.org
	#define MQTT_BROKER		"test.mosquitto.org" // test.mosquitto.org
	#define MQTT_PASS 		"try"
	#define MQTT_PORT  			1883	// use 8883 for SSL
	#define MQTT_USER			"sircoolio"
#endif

// Configuration Step 4: Influxdb credentials
// set Influxdb login and storage parameters
#ifdef INFLUX
	// InfluxDB v2.X
	#define INFLUX_V2 
	#define INFLUXDB_URL "http://192.168.1.27:8086"
	#define INFLUXDB_TOKEN "YoXD17r1o9pDCz2UcnPlqbjndAyEDEAsG__sJISktsMEHRZUXsaxNMFbZtN3-WTDQ4VbpzR65klds-A59X5Y4w=="
	#define INFLUXDB_ORG "kvp"
	#define INFLUXDB_BUCKET "7828"
#endif

// Configuration Step 5: Set key device and installation configuration parameters.  These are used
// widely throughout the code to properly identify the device and generate important
// operating elements like MQTT topics, InfluxDB data tags (metadata).  Should be
// customized to match the target installation. Values here are examples.
#define DEVICE           "rco2"
#define DEVICE_SITE      "7828"
#define DEVICE_LOCATION  "inside"
#define DEVICE_ROOM      "lab"
#define DEVICE_ID        "rco2-dev"