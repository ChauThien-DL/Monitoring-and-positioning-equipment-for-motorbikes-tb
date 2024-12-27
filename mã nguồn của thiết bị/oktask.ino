#include <ArduinoJson.h>
#define TINY_GSM_MODEM_SIM7600
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_task_wdt.h"

#include <math.h>




Adafruit_MPU6050 mpu;

#define simen 15
#define in_key 33
#define out_key 23
#define find_pin 18
#define buzzer_pin 19
#define bk_pin 13

#define ADC_pin 32

#define d0 12
#define d1 14
#define d2 27
#define d3 26
#define vt 25

#define rung 35



uint8_t signal_ctr = 0;
uint8_t signal_ctr_w = 0;
uint8_t signal_ctr_w1 = 0;
uint8_t vibra = 0;

uint8_t antithef_send = 0;

uint8_t state_active = 1;
uint8_t state_find = 0;
uint8_t state_buzzer = 0;
uint8_t state_de = 0;
uint8_t state_mode = 0;
uint8_t state_warning = 0;
uint8_t state_key = 0;

uint8_t state_ace = 0;

uint8_t state_d0 = 0;
uint8_t state_d1 = 0;
uint8_t state_d2 = 0;
uint8_t state_d3 = 0;




const char* mqtt_server = "52.74.168.92";
const int mqtt_port = 1883;
const char* mqtt_user = "dinhvixemay";
const char* mqtt_pass = "123";
const char* mqtt_client_id = "xemay";
const char* gps_topic = "GPS";

HardwareSerial sim7600(1); 
TinyGsm modem(sim7600);
TinyGsmClient simClient(modem);
PubSubClient mqttClient(simClient);

uint32_t ctime_warning =0;
uint32_t ctime_warning1 =0;
uint32_t ctime_gps = 0;
uint32_t ctime_se = 0;
uint32_t ctime_remote = 0;
uint32_t ctime_signal =0;
uint32_t ctime_mqtt_reconnect = 0;
uint32_t ctime_mqtt_state = 0;
uint8_t o_key = 0;

int8_t x, y, z;
uint16_t roll_send = 0;
uint16_t pitch_send = 0;
uint8_t temp_mpu;

float vol;
String gpsInfo = "";

void setup() {
    Serial.begin(115200);
    sim7600.begin(115200, SERIAL_8N1, 16, 17); 
    delay(500);
    
    analogReadResolution(12);

    pinMode(simen, OUTPUT);
    pinMode(in_key, INPUT);
    pinMode(out_key, OUTPUT);
    pinMode(find_pin, OUTPUT);
    pinMode(buzzer_pin, OUTPUT);
    pinMode(bk_pin, OUTPUT);

    pinMode(d0, INPUT);
    pinMode(d1, INPUT);
    pinMode(d2, INPUT);
    pinMode(d3, INPUT);
    pinMode(vt, INPUT);
    pinMode(rung , INPUT);

    if (!mpu.begin()) {
        Serial.println("MPU6050 initialization failed!");
      
    }

    sim7600.println("AT");
    delay(1000);
    while (sim7600.available()) {
      Serial.write(sim7600.read());
    }

    sim7600.println("AT+CPIN?");
    delay(1000);
    while (sim7600.available()) {
      Serial.write(sim7600.read());
    }

    sim7600.println("AT+COPS?");
    delay(1000);
    while (sim7600.available()) {
      Serial.write(sim7600.read());
    }

    sim7600.println("AT+CGPS=1");
    delay(1000);
    while (sim7600.available()) {
      Serial.write(sim7600.read());
    }
    
    delay(2000);


    xTaskCreate(task1, "Task1", 15360, NULL, 2, NULL);
    xTaskCreate(task2, "Task2", 15360, NULL, 1, NULL);
    
}

void task1(void *pvParameters) {
    esp_task_wdt_add(NULL);  
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool isConnected4G = false;
    bool isConnectedMQTT = false;

    
    double lat1, lon1, lat2, lon2, lat3, lon3;
    unsigned long lastMillis_dis = 0;
    uint8_t count = 0;

    String CR_Time;
    double lat, lon;
    uint8_t speed;
    uint8_t spd;

    double totalDistance = 0;
    uint8_t state_distance = 0;

    while (true) {
      delay(500);
        if (!isConnected4G) {
            while (!isConnected4G) { 
                Serial.println("Attempting to connect to 4G...");
                isConnected4G = connect4G();
          
                esp_task_wdt_reset();
                vTaskDelay(500 / portTICK_PERIOD_MS);  
            }
            Serial.println("Connected to 4G.");
        }

        if (!isConnectedMQTT) {
            while (!isConnectedMQTT) {
                Serial.println("Attempting to connect to MQTT...");
                mqttClient.setServer(mqtt_server, mqtt_port);
                mqttClient.setCallback(mqttCallback);
                isConnectedMQTT = reconnectMQTT();
            
                esp_task_wdt_reset();
                vTaskDelay(500 / portTICK_PERIOD_MS);  
            }
            Serial.println("Connected to MQTT.");
        }

    
        if (isConnected4G && isConnectedMQTT) {
            while (mqttClient.connected()) {
       
                mqttClient.loop();
                CR_Time = getCurrentTime();
                if (CR_Time == "Invalid Time") {
                    Serial.println("Error: Invalid Time Format");
                    break;
                }

                if (millis() - ctime_gps >= 20000) {
                    if(lon != 0){
                        sendFullDataMQTT(CR_Time ,x , y, z, pitch_send, temp_mpu, o_key, vibra, state_warning, totalDistance, vol, lat, lon, speed);
                    }
                    requestGPSData();
                    esp_task_wdt_reset();
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    ctime_gps = millis();
                }
                readGPSData();
                esp_task_wdt_reset();

                if (gpsInfo.length() > 0 && !gpsInfo.startsWith("+CGPSINFO: ,,,,,,,,") && gpsInfo.indexOf(",") != -1) {
                    Serial.println("GPS Raw Data:");
                    Serial.println(gpsInfo);

                    if (millis() - lastMillis_dis >= 20000) {
                        lastMillis_dis = millis();
                      
                        getGPSData(gpsInfo, lat, lon, spd);
                        speed = spd * 1.852 ; 
                        
                        if (count == 0) {
                            lat1 = lat;
                            lon1 = lon;
                        } else if (count == 1) {
                            lat2 = lat;
                            lon2 = lon;
                        } else if (count == 2) {
                            lat3 = lat;
                            lon3 = lon;

                            double distance1 = haversine(lat1, lon1, lat2, lon2);
                            double distance2 = haversine(lat2, lon2, lat3, lon3);
                            totalDistance = (distance1 + distance2) * 1000;
                            totalDistance = round(totalDistance * 100) / 100;

                            Serial.print("Quãng đường giữa ba tọa độ: ");
                            Serial.print(totalDistance);
                            Serial.println(" m");

                            state_distance = (totalDistance < 5) ? 0 : (totalDistance <= 15 ? 1 : 2);

                            count = -1;
                        }

                        count++;

                        sendDataTrain(CR_Time, state_warning, o_key, vibra, state_ace, state_distance, speed);
                    }

                    
                    
                    String formattedGPSData = getFormattedGPSData(gpsInfo, CR_Time);
                    
                    if (formattedGPSData.length() > 0) {
                        Serial.print("Formatted GPS Data: ");
                        Serial.println(formattedGPSData);
                        sendDataToMQTT(formattedGPSData, gps_topic);
                    } else {
                        Serial.println("Formatted GPS Data is empty or invalid.");
                    }
                    
                    esp_task_wdt_reset();
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    gpsInfo = "";
                }

                vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
                esp_task_wdt_reset();
            }

            Serial.println("MQTT disconnected, retrying...");
            isConnectedMQTT = false;
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        } 
       
    }
}

void task2(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        if (millis() - ctime_se >= 100) {
            uint8_t state_ik = digitalRead(in_key);
            uint16_t adc = analogRead(ADC_pin);

            float vol_esp32 = ((float)adc / 4095.0) * 3.3;
            vol = vol_esp32 * ((30500.0 + 7000.0) / 7000.0);

            vol = round(vol * 100) / 100;
            state_key = (state_ik == 1) ? 0 : 1;
            vibra = digitalRead(rung);
            ctime_se = millis();
        }

        if (millis() - ctime_remote >= 50) {
            state_d0 = digitalRead(d0);
            state_d1 = digitalRead(d1);
            state_d2 = digitalRead(d2);
            state_d3 = digitalRead(d3);
            ctime_remote = millis();
        }

        if (millis() - ctime_mqtt_state >= 1000) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            x = a.acceleration.x;
            y = a.acceleration.y;
            z = a.acceleration.z;
            temp_mpu = temp.temperature;

            uint16_t pitch = atan2(-y, sqrt(x * x + z * z)) * 180.0 / PI;

            pitch_send = abs(pitch);

            if ( pitch_send < 41) {
                state_ace = 1;

            } else {
                state_ace = 0;
            }


  
            vTaskDelay(50 / portTICK_PERIOD_MS);  
            ctime_mqtt_state = millis();
        }

        checkStates();
        
        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);  
    }
}

void loop() {
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0; 
    
    double lat1Rad = lat1 * (M_PI / 180.0);
    double lon1Rad = lon1 * (M_PI / 180.0);
    double lat2Rad = lat2 * (M_PI / 180.0);
    double lon2Rad = lon2 * (M_PI / 180.0);
    
    double dlat = lat2Rad - lat1Rad;
    double dlon = lon2Rad - lon1Rad;
    
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1Rad) * cos(lat2Rad) *
               sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    return R * c; 
}

bool connect4G() {
    Serial.println("Attempting to connect to 4G...");
    for (uint8_t i = 0; i < 3; i++) {
        if (modem.gprsConnect("v-internet", "", "")) {
            Serial.println("4G connected");
            return true;
        } else {
            Serial.println("4G connection failed, retrying...");
            modem.gprsDisconnect();
            delay(1000);
            modem.gprsConnect("v-internet", "", "");
            esp_task_wdt_reset(); 
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    Serial.println("4G connection failed after 10 attempts.");
    return false;
}


bool reconnectMQTT() {
    const uint8_t maxAttempts = 3; 
    for (uint8_t attempt = 0; attempt < maxAttempts; attempt++) {
        Serial.print("Connecting to MQTT... (Attempt ");
        Serial.print(attempt + 1);
        Serial.println(")");
        
        if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_pass)) {
            Serial.println("MQTT connected");
            mqttClient.subscribe("state_active");
            mqttClient.subscribe("state_find");
            mqttClient.subscribe("state_buzzer");
            mqttClient.subscribe("state_de");
            mqttClient.subscribe("state_mode");
            mqttClient.subscribe("state_warning");
            return true;
        } else {
            Serial.print("MQTT connection failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(". Retrying in 5 seconds...");
            esp_task_wdt_reset(); 
            vTaskDelay(5000 / portTICK_PERIOD_MS); 
        }
    }
    Serial.println("MQTT connection failed after maximum attempts.");
    return false; 
}

void requestGPSData() {
  sim7600.println("AT+CGPSINFO");
  gpsInfo = ""; 
}

void readGPSData() {
  while (sim7600.available()) {
    char c = sim7600.read();
    gpsInfo += c;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);

  if (String(topic) == "state_active") {
    state_active = message.toInt();
  } else if (String(topic) == "state_find") {
    state_find = message.toInt();
  } else if (String(topic) == "state_buzzer") {
    state_buzzer = message.toInt();
  } else if (String(topic) == "state_de") {
    state_de = message.toInt();
  } else if (String(topic) == "state_mode") {
    state_mode = message.toInt();
  } else if (String(topic) == "state_warning") {
    state_warning = message.toInt();
  }
}

void checkStates() {

    digitalWrite(out_key, (state_active == 0) ? HIGH : LOW);  

    o_key = (state_active == 1 && state_key == 1) ? 1 : 0;
    if ((state_find == 1 && state_d3 == 0) || 
        (state_find == 0 && state_d3 == 1) || 
        (state_find == 1 && state_d3 == 1)) {
        if (millis() - ctime_signal >= 300) {
            signal_ctr = !signal_ctr;
            digitalWrite(find_pin, signal_ctr);
            ctime_signal = millis();
        }
        if ((state_buzzer == 1 && state_d1 == 0) || 
            (state_buzzer == 0 && state_d1 == 1) || 
            (state_buzzer == 1 && state_d1 == 1)) {
            digitalWrite(buzzer_pin, HIGH);
        } else {
            digitalWrite(buzzer_pin, LOW);
        }
    } else {
        digitalWrite(find_pin, LOW);
        digitalWrite(buzzer_pin, LOW);
    }
    if ((state_de == 1 && state_d0 == 0) || 
        (state_de == 0 && state_d0 == 1) || 
        (state_de == 1 && state_d0 == 1)) {
        digitalWrite(bk_pin, HIGH);
    } else {
        digitalWrite(bk_pin, LOW);
    }
}


void getGPSData(String gpsInfo, double &latitude, double &longitude, uint8_t &speed) {
    if (gpsInfo.indexOf("+CGPSINFO:") != -1) {
        uint8_t latStart = gpsInfo.indexOf(":") + 1;
        uint8_t latEnd = gpsInfo.indexOf(",", latStart);

        String latDDM = gpsInfo.substring(latStart, latEnd);
        if (latDDM.length() == 0) {
            Serial.println("No valid GPS data found.");
            latitude = 0.0;
            longitude = 0.0;
            speed = 0;
            return; 
        }

        char latDirection = gpsInfo.charAt(latEnd + 1);

        uint8_t lonStart = latEnd + 3;
        uint8_t lonEnd = gpsInfo.indexOf(",", lonStart);
        String lonDDM = gpsInfo.substring(lonStart, lonEnd);
        if (lonDDM.length() == 0) {
            Serial.println("No valid GPS data found.");
            latitude = 0.0;
            longitude = 0.0;
            speed = 0;
            return; 
        }
        char lonDirection = gpsInfo.charAt(lonEnd + 1);

        uint8_t speedStart = gpsInfo.indexOf(",", lonEnd) + 1;
        uint8_t speedEnd = gpsInfo.indexOf(",", speedStart);
        String speedStr = gpsInfo.substring(speedStart, speedEnd);

        double latDegrees = latDDM.substring(0, 3).toFloat(); 
        double latMinutes = latDDM.substring(3).toFloat(); 
        latitude = latDegrees + (latMinutes / 60.0);
        if (latDirection == 'S') latitude = -latitude;

        double lonDegrees = lonDDM.substring(0, 3).toFloat(); 
        double lonMinutes = lonDDM.substring(3).toFloat(); 
        longitude = lonDegrees + (lonMinutes / 60.0);
        if (lonDirection == 'W') longitude = -longitude;

        latitude = round(latitude * 1000000.0) / 1000000.0;
        longitude = round(longitude * 1000000.0) / 1000000.0;

        speed = speedStr.toInt();
    } else {
        Serial.println("No valid GPS data found.");
        latitude = 0.0;
        longitude = 0.0;
        speed = 0;
    }
}


String getFormattedGPSData(String gpsInfo, String currentTime) {
  if (gpsInfo.indexOf("+CGPSINFO:") != -1) {
    uint8_t latStart = gpsInfo.indexOf(":") + 1;
    uint8_t latEnd = gpsInfo.indexOf(",", latStart);

    String latDDM = gpsInfo.substring(latStart, latEnd);
    if (latDDM.length() == 0) {
      Serial.println("No valid GPS data found.");
      return ""; 
    }

    char latDirection = gpsInfo.charAt(latEnd + 1);

    uint8_t lonStart = latEnd + 3;
    uint8_t lonEnd = gpsInfo.indexOf(",", lonStart);
    String lonDDM = gpsInfo.substring(lonStart, lonEnd);
    if (lonDDM.length() == 0) {
      Serial.println("No valid GPS data found.");
      return ""; 
    }
    char lonDirection = gpsInfo.charAt(lonEnd + 1);

    uint8_t speedStart = gpsInfo.indexOf(",", lonEnd) + 1;
    uint8_t speedEnd = gpsInfo.indexOf(",", speedStart);
    String speed = gpsInfo.substring(speedStart, speedEnd);

    double latDegrees = latDDM.substring(0, 3).toFloat();
    double latMinutes = latDDM.substring(3).toFloat();
    double latDecimal = latDegrees + (latMinutes / 60.0);
    if (latDirection == 'S') latDecimal = -latDecimal;

    double lonDegrees = lonDDM.substring(0, 3).toFloat();
    double lonMinutes = lonDDM.substring(3).toFloat(); 
    double lonDecimal = lonDegrees + (lonMinutes / 60.0);
    if (lonDirection == 'W') lonDecimal = -lonDecimal;

    latDecimal = round(latDecimal * 1000000.0) / 1000000.0;
    lonDecimal = round(lonDecimal * 1000000.0) / 1000000.0;

    StaticJsonDocument<200> doc;
    doc["time"] = currentTime; 
    doc["device"] = "Exciter";
    doc["lat"] = latDecimal;
    doc["long"] = lonDecimal;
    doc["speed"] = speed.toFloat();

    String jsonString;
    serializeJson(doc, jsonString);

    return jsonString;
  }

  return "";
}



String getCurrentTime() {
  sim7600.println("AT+CCLK?");
  String timeInfo = "";
  vTaskDelay(500 / portTICK_PERIOD_MS); 
  while (sim7600.available()) {
    char c = sim7600.read();
    timeInfo += c;
  }
  int start = timeInfo.indexOf("\"") + 1;
  int end = timeInfo.indexOf("+", start);
  if (end == -1) {
    end = timeInfo.indexOf("\"", start);
  }
  String rawTime = timeInfo.substring(start, end);

  int commaIndex = rawTime.indexOf(',');
  String datePart = rawTime.substring(0, commaIndex);
  String timePart = rawTime.substring(commaIndex + 1);

  String formattedDate = "20" + datePart.substring(0, 2) + "-" + datePart.substring(3, 5) + "-" + datePart.substring(6, 8);
  String formattedTime = timePart.substring(0, 2) + ":" + timePart.substring(3, 5) + ":" + timePart.substring(6, 8);
  
  return formattedDate + "T" + formattedTime + "Z";
}


void sendDataToMQTT(String data, const char* topic) {
  Serial.print("Sending data to topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(data);

  if (mqttClient.publish(topic, data.c_str())) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Failed to send data");
  }
}


void sendDataTrain(String currentTime, uint8_t ModeAntiTheft, uint8_t o_key, uint8_t vib, uint8_t ace, uint8_t Distance, uint8_t speed) {
  StaticJsonDocument<200> doc;
  doc["time"] = currentTime;
  doc["device"] = "Exciter";
  doc["Mode_AntiTheft"] = ModeAntiTheft; 
  doc["key"] = o_key;
  doc["Vibration"] = vib;
  doc["Acceleration"] = ace;    
  doc["Distance"] = Distance;   
  doc["Speed"] = speed;      

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.print("Sending Data Train to topic dataTrain: ");
  Serial.println(jsonString);

  if (!mqttClient.connected()) {
    Serial.println("MQTT client not connected, retrying...");
    reconnectMQTT(); 
    if (!mqttClient.connected()) {
      vTaskDelay(100 / portTICK_PERIOD_MS); 
      return; 
    }
  }

  if (mqttClient.publish("dataTrain", jsonString.c_str())) {
    Serial.println("Data Train sent successfully");
  } else {
    Serial.println("Failed to send Data Train");
  }
}


void sendFullDataMQTT(String currentTime, int8_t x, int8_t y, int8_t z, uint16_t pitch, uint8_t temp_mpu, uint8_t o_key, uint8_t vibra, uint8_t Mode_AntiTheft, double totalDis, float vol_acquy, double lat, double lon, uint8_t speed ) {

  StaticJsonDocument<200> doc;
  doc["time"] = currentTime;
  doc["device"] = "Exciter"; 
  doc["x"] = x;
  doc["y"] = y;
  doc["z"] = z;
  doc["Rotation"] = pitch;
  doc["temp"] = temp_mpu;
  doc["o_key"] = o_key;
  doc["Vibration"] = vibra;
  doc["Mode_AntiTheft"] = Mode_AntiTheft;
  doc["TotalDistance"] = totalDis;
  doc["Vol_Acquy"] = vol_acquy;
  doc["lat"] = lat;
  doc["long"] = lon;
  doc["Speed"] = speed;

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.print("Sending full data to topic fulldata: ");
  Serial.println(jsonString);

  if (!mqttClient.connected()) {
      Serial.println("MQTT client not connected, retrying...");
      reconnectMQTT();
      if (!mqttClient.connected()) {
          vTaskDelay(100 / portTICK_PERIOD_MS); 
          return; 
      }
  }


  if (mqttClient.publish("fulldata", jsonString.c_str())) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Failed to send data");
  }
}

