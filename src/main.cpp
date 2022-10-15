#ifdef ESP8266
#include <ESP8266WiFiMulti.h>
#endif
#ifdef ESP32
#include <WiFiMulti.h>
#endif
#include <PubSubClient.h>
#include "localconfig.h" // Empty file by default

#ifndef WIFI_SSID
#define WIFI_SSID "DEMO_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "DEMO_PASSPHRASE"
#endif

#ifndef MQTT_TOPIC_BASE_RAIN
#define MQTT_TOPIC_BASE_RAIN "exp/NX6331"
#endif

#ifndef MQTT_TOPIC_BASE_F007TP
#define MQTT_TOPIC_BASE_F007TP "exp/F007TP-"
#endif

#ifndef MQTT_CLIENT
#define MQTT_CLIENT "NX6331_F007TP_ESP"
#endif

#ifndef MQTT_SERVER
#define MQTT_SERVER MQTTserver(192, 168, 99, 99);
#endif

#define DEBUG 0

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
IPAddress MQTT_SERVER;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

#define countof(x) (sizeof(x)/sizeof(x[0]))
// Interface Definitions
#ifdef ESP8266
#define RXPIN D2 //The number of signal from the Rx
String ChipId = "esp8266_" + String(ESP.getChipId(), HEX);
#endif
#ifdef ESP32
#define RXPIN27 //The number of signal from the Rx
String ChipId = "esp32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
#endif

unsigned long t;

// Connect to Wifi and report IP address
void start_Wifi()
{
    #ifdef DEBUG
    Serial.println("Starting Wifi");
    #endif
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.begin(ssid, password);
    int counter = 0;
    while ((WiFi.status() != WL_CONNECTED) && (counter < 200))
    {
        delay(500);
        counter ++;
        #if defined(DEBUG)
        Serial.print(".");
        #endif
    }
    if (counter >= 200)
    {
        #ifdef DEBUG
        Serial.println("Connection Failed! Rebooting...");
        #endif
        ESP.restart();
    }
    #ifdef DEBUG
    Serial.println("");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    #endif
    WiFi.setAutoReconnect(true);
}

void publish (String topic, String payload)
{
    if (!mqtt_client.connected())
    {
        mqtt_client.connect(MQTT_CLIENT);
        #ifdef DEBUG
            Serial.println("MQTT Connect");
        #endif
    }

    mqtt_client.publish(topic.c_str(), payload.c_str());
}

#define RAIN_SYNC_PULSE_MIN 7300
#define RAIN_SYNC_PULSE_MAX 7700
#define RAIN_SHORT_PULSE_MIN 900
#define RAIN_SHORT_PULSE_MAX 1100
#define RAIN_LONG_PULSE_MIN 2500
#define RAIN_LONG_PULSE_MAX 2900

#define F007TP_SHORT_PULSE_MIN 350
#define F007TP_SHORT_PULSE_MAX 600
#define F007TP_LONG_PULSE_MIN 1350
#define F007TP_LONG_PULSE_MAX 1600


////// Rain Gauge

uint8_t rain_checksum(int length, uint8_t *buff)
{
    uint8_t checksum = 0;

    for ( int byteCnt = 0; byteCnt < length; byteCnt++)
    {
        checksum += buff[byteCnt];
    }

    return checksum;
}

void rain_gauge(uint16_t pulses[])
{
    uint8_t data[8];
    for (int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            data[i] <<= 1;
            unsigned long d = pulses[i * 8 + j];
            if ((d > RAIN_SHORT_PULSE_MIN) and (d < RAIN_SHORT_PULSE_MAX))
                data[i] ^= 1;
            else if ((d < RAIN_LONG_PULSE_MIN) or (d > RAIN_LONG_PULSE_MAX))
                return;
        }
    }

    if (rain_checksum(7, data) != data[7])
        return;

    // ID : Bytes 0 & 1 Little Endian
    uint16_t id = (data[0] << 8) + data[1];
    #ifdef DEBUG
        Serial.print("Val:");
        Serial.println((data[4] << 8) + data[3]);
    #endif
    uint16_t val = (data[4] << 8) + data[3] + 10;
    float rainc = int(1.08f * val);
    
    // Temp : Bytes 5 & 6 Little Endian in 10th of °F + offset 900
    uint16_t tf10 = (data[6] << 8) + data[5] - 900;
    float tc = (tf10 - 320) / 18.0f;
    
    // Reset : Byte 2 - bit 6
    /* Serial.print("Reset:");
       Serial.println(data[2] & 64); */

    // Battery Low : Byte 2 - bit 7
    /* Serial.print("Battery Low:");
       Serial.println(data[2] & 128); */

    String source = "\"id\": " + String(id) + ", \"source\": \"" + ChipId + "\"";

    String topic = MQTT_TOPIC_BASE_RAIN + String("/data/temperature");
    String payload = "{\"value\": " + String(tc, 1) + ", " + source + "}";
    publish(topic, payload);
                
    topic = MQTT_TOPIC_BASE_RAIN + String("/data/rain");
    payload = "{\"value\": " + String(rainc, 0) + ", " + source + "}";
    publish(topic, payload);

    topic = MQTT_TOPIC_BASE_RAIN + String("/data/low_battery");
    payload = "{\"value\": " + String(data[2] >> 7) + ", " + source + "}";
    publish(topic, payload);

    delay(1000);
 
}

////// F007TP

uint8_t f007tp_checksum(int length, uint8_t *buff)
{
    uint8_t checksum = 0;

    for ( int byteCnt = 0; byteCnt < length; byteCnt++)
    {
        checksum += buff[byteCnt];
    }

    return checksum;
}

void f007tp(uint16_t pulses[])
{
    uint8_t data[40];
    data[0] = 0;

    for (int i = 0; i < 39; i++)
    {
        unsigned long d = pulses[i];
        if ((d > F007TP_SHORT_PULSE_MIN) and (d < F007TP_SHORT_PULSE_MAX))  // 1
            data[i + 1] = 1;
        else if ((d > F007TP_LONG_PULSE_MIN) and (d < F007TP_LONG_PULSE_MAX))  // 0
            data[i + 1] = 0;
        else
            return;
    }

    if (data[0] or (!data[1]) or data[2] or data[3])  // Should be 0100
        return;

    // Calculate and then CRC
    uint8_t crc = 0;
    for (int i=0; i<32; i++)
    {
        uint8_t mix = (crc >> 7) ^ data[i];
        crc <<= 1;
        if (mix)
            crc ^= 0x31;
    }
    for (int i=0; i<8; i++)
    {
        if ((crc >> 7) != data[32 + i])
           return;
        crc <<= 1;
    }

    uint8_t id = (data[5] << 2) + (data[6] << 1) + data[7] + 1;
    #ifdef DEBUG
        Serial.print("ID: ");
        Serial.println(id);
    #endif
    uint16_t temperature = 0;
    for (int i=13; i<24; i++)
    {
        temperature <<=1;
        if (data[i])
            temperature |= 1;
    }
    #ifdef DEBUG
        Serial.print("Temperature: ");
        if (data[12])
            Serial.print('-');
        Serial.println(temperature);
    #endif

    String sign = (data[12])? "-" : "";
    String topic = MQTT_TOPIC_BASE_F007TP + String(id) + String("/data/temperature");
    String payload = "{\"value\": " + sign + String(temperature / 10) + "." + String(temperature % 10) + ", \"source\": \"" + ChipId + "\"}";
    publish(topic, payload);

    // Hygrometry in data[24] -> data[31]

}

void record()
{
    uint8_t protocol = 0;
    uint8_t synchro = 0;
    uint16_t pulses[64];
    for (int i = 0; i < 10000; i++)
    {
        unsigned long d = pulseIn(RXPIN, HIGH, 10000);
        // Synchronising : theorically 8 (but 4/5 in practice?) short pulses (aka "1") followed by 1 long (aka "0")
        if ((d > F007TP_SHORT_PULSE_MIN) and (d < F007TP_SHORT_PULSE_MAX))  // 1
            synchro++;
        else if ((d > F007TP_LONG_PULSE_MIN) and (d < F007TP_LONG_PULSE_MAX))  // 0
        {
            if (synchro > 3)
            {
            
                protocol = 2;
                break;
            }
            synchro = 0;
        } else
        {
           if ((d > RAIN_SYNC_PULSE_MIN) and (d < RAIN_SYNC_PULSE_MAX))
            {
                protocol = 1;
                break;
            }
            synchro = 0;
        }
    }
    
    if (protocol == 1)
    // Protocol 1 = rain_gauge
    {
        for (int i = 0; i < 64; i++)
            pulses[i] = pulseIn(RXPIN, HIGH);
        rain_gauge(pulses);
    } else if (protocol == 2)
    // Protocol 2 = F007TP
    {
        for (int i = 0; i < 40; i++)
            pulses[i] = pulseIn(RXPIN, HIGH);
            f007tp(pulses);
    }
}

void setup()
{
    Serial.begin(115200);

    start_Wifi();
    mqtt_client.setServer(MQTTserver, 1883);

    String payload = "{\"ip\": \"" + WiFi.localIP().toString() + "\"" + ", \"source\": \"" + ChipId + "\"}";
    publish(MQTT_TOPIC_BASE_RAIN + String("/data/info"), payload);
      
    pinMode(RXPIN, INPUT);
    t = millis();
}

void loop()
{
    #ifdef DEBUG
    if (millis() > t + 10000)
    {
        Serial.println("--check--");
        t = millis(); 
    }
    #endif
    record();

} //end of mainloop



