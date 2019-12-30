/**The MIT License (MIT)

  Copyright (c) 2018 by Daniel Eichhorn - ThingPulse

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  See more at https://thingpulse.com
*/
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ESPWiFi.h>
#include <ESPHTTPClient.h>
#include <JsonListener.h>
#include <time.h>                       // time() ctime()
#include <sys/time.h>                   // struct timeval
#include <coredecls.h>                  // settimeofday_cb()
#include <ArduinoOTA.h>
#include "SSD1306Wire.h"
#include "OLEDDisplayUi.h"
#include "Wire.h"
#include "OpenWeatherMapCurrent.h"
#include "OpenWeatherMapForecast.h"
#include "WeatherStationFonts.h"
#include "WeatherStationImages.h"
#include <DHT.h>                 // For DHT22 temperature and humidity sensor
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>         //https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>        //https://github.com/knolleary/pubsubclient
#include <MD5Builder.h>

#define TZ              1       // (utc+) TZ in hours
#define DST_MN          60      // use 60mn for summer time in some countries
#define TZ_MN           ((TZ)*60)
#define TZ_SEC          ((TZ)*3600)
#define DST_SEC         ((DST_MN)*60)
#define DHTPIN  2
#define DHTTYPE DHT22
#define HOME_ASSISTANT_DISCOVERY 1

// Setup
const int UPDATE_INTERVAL_SECS = 20 * 60; // Update every 20 minutes

// Display Settings
const int I2C_DISPLAY_ADDRESS = 0x3c;

const int SDA_PIN = 4; 
const int SDC_PIN = 5; 

// OpenWeatherMap Settings
// Sign up here to get an API key:
// https://docs.thingpulse.com/how-tos/openweathermap-key/
String OPEN_WEATHER_MAP_APP_ID = "4c518e60b4cf8de0b50ade27d5a7408c";
/*
  Go to https://openweathermap.org/find?q= and search for a location. Go through the
  result set and select the entry closest to the actual location you want to display
  data for. It'll be a URL like https://openweathermap.org/city/2657896. The number
  at the end is what you assign to the constant below.
*/
String OPEN_WEATHER_MAP_LOCATION_ID = "727011";

// Pick a language code from this list:
// Arabic - ar, Bulgarian - bg, Catalan - ca, Czech - cz, German - de, Greek - el,
// English - en, Persian (Farsi) - fa, Finnish - fi, French - fr, Galician - gl,
// Croatian - hr, Hungarian - hu, Italian - it, Japanese - ja, Korean - kr,
// Latvian - la, Lithuanian - lt, Macedonian - mk, Dutch - nl, Polish - pl,
// Portuguese - pt, Romanian - ro, Russian - ru, Swedish - se, Slovak - sk,
// Slovenian - sl, Spanish - es, Turkish - tr, Ukrainian - ua, Vietnamese - vi,
// Chinese Simplified - zh_cn, Chinese Traditional - zh_tw.
String OPEN_WEATHER_MAP_LANGUAGE = "bg";
const uint8_t MAX_FORECASTS = 4;

const boolean IS_METRIC = true;

// Adjust according to your language
const String WDAY_NAMES[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
const String MONTH_NAMES[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
 
WiFiClient espClient;   // MQTT
DHT dht(DHTPIN, DHTTYPE);
PubSubClient mqttClient(espClient);

//flag for saving data
bool shouldSaveConfig = false;

float temperatureCoef = 0.86;

float dhtTemperature = 0;
float dhtHumidity = 0;

unsigned long sensorPreviousMillis = 0;
const long sensorInterval = 30000;

unsigned long mqttConnectionPreviousMillis = millis();
const long mqttConnectionInterval = 60000;

char mqtt_server[40] = "iot.eclipse.org";
char mqtt_port[6] = "1883";
char workgroup[32] = "workgroup";
char username[20] = ""; // MQTT username and password
char password[20] = "";
#ifdef HOME_ASSISTANT_DISCOVERY
char ha_name[32 + 1] = "";      // Make sure the machineId fits.
#endif
char temp_scale[40] = "celsius";

// Set the temperature in Celsius or Fahrenheit
// true - Celsius, false - Fahrenheit
bool configTempCelsius = true;

// Configure pins
const int pinAlarm = 16;
const int pinButton = 0;

// MD5 of chip ID.  If you only have a handful of thermometers and use
// your own MQTT broker (instead of iot.eclips.org) you may want to
// truncate the MD5 by changing the 32 to a smaller value.
char machineId[32 + 1] = "";

char cmnd_temp_format[16 + sizeof(machineId)];

// Initialize the oled display for address 0x3c
// sda-pin=14 and sdc-pin=12
SSD1306Wire     display(I2C_DISPLAY_ADDRESS, SDA_PIN, SDC_PIN);
OLEDDisplayUi   ui( &display );

OpenWeatherMapCurrentData currentWeather;
OpenWeatherMapCurrent currentWeatherClient;

OpenWeatherMapForecastData forecasts[MAX_FORECASTS];
OpenWeatherMapForecast forecastClient;

time_t now;

//declaring prototypes
void drawProgress(OLEDDisplay *display, int percentage, String label);
void updateData(OLEDDisplay *display);
void drawDateTime(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void drawHomeCond(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void drawCurrentWeather(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void drawForecast(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void drawForecastDetails(OLEDDisplay *display, int x, int y, int dayIndex);
void drawHeaderOverlay(OLEDDisplay *display, OLEDDisplayUiState* state);
void setReadyForWeatherUpdate();
void readSensors();
void setReadyForWeatherUpdate();
void otaStarted();
void otaFinished();
void otaProgress(unsigned int progress, unsigned int total);
void otaError(ota_error_t error);
void factoryReset();
void waitForFactoryReset() ;
void mqttCallback(char* topic, byte * payload, unsigned int length);
void calculateMachineId();
void publishSensorData(const char* subTopic, const char* key, const float value);
void publishSensorData(const char* subTopic, const char* key, const String & value);
void saveConfigCallback();
void saveConfig();
void processMessageScale(const char* text);
bool publishSensorDiscovery(const char *config_key, const char *device_class, const char *name_suffix, 
                            const char *state_topic, const char *unit, const char *value_template);
void publishState();
float convertCelsiusToFahrenheit(float temperature);
float convertTemperature(float temperature);
String formatTemperature(float temperature);
void setMQTTtopics();

// flag changed in the ticker function every 10 minutes
bool readyForWeatherUpdate = false;

String lastUpdate = "--";

long timeSinceLastWUpdate = 0;

// Add frames
// this array keeps function pointers to all frames
// frames are the single views that slide from right to left
FrameCallback frames[] = { drawDateTime, drawHomeCond, drawCurrentWeather, drawForecast };
int numberOfFrames = 4;

OverlayCallback overlays[] = { drawHeaderOverlay };
int numberOfOverlays = 1;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  // initialize dispaly
  display.init();
  display.clear();
  display.display();

  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setContrast(255);

  dht.begin();

  waitForFactoryReset();

  calculateMachineId();

  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        const size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        if (DeserializationError::Ok == deserializeJson(json, buf.get()))
        {
          serializeJson(json, Serial);
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(workgroup, json["workgroup"]);
          strcpy(username, json["username"]);
          strcpy(password, json["password"]);
          {
            const char *s = json["temp_scale"];
            if (!s)
              s = "celsius";
            strcpy(temp_scale, s);
          }
#ifdef HOME_ASSISTANT_DISCOVERY
          {
            const char *s = json["ha_name"];
            if (!s)
              s = machineId;
            snprintf(ha_name, sizeof(ha_name), "%s", s);
          }
#endif
        }
        else
        {
          Serial.println("failed to load json config");
        }
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }

  //end read
  setMQTTtopics();

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, sizeof(mqtt_server));
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, sizeof(mqtt_port));
  WiFiManagerParameter custom_workgroup("workgroup", "workgroup", workgroup, sizeof(workgroup));
  WiFiManagerParameter custom_mqtt_user("user", "MQTT username", username, sizeof(username));
  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", password, sizeof(password));
  WiFiManagerParameter custom_temperature_scale("temp_scale", "Temperature scale", temp_scale, sizeof(temp_scale));
#ifdef HOME_ASSISTANT_DISCOVERY
  WiFiManagerParameter custom_mqtt_ha_name("ha_name", "Sensor name for Home Assistant", ha_name, sizeof(ha_name));
#endif

  char htmlMachineId[200];
  sprintf(htmlMachineId, "<p style=\"color: red;\">Machine ID:</p><p><b>%s</b></p><p>Copy and save the machine ID because you will need it to control the device.</p>", machineId);
  WiFiManagerParameter custom_text_machine_id(htmlMachineId);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_workgroup);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_temperature_scale);
  wifiManager.addParameter(&custom_text_machine_id);
#ifdef HOME_ASSISTANT_DISCOVERY
  wifiManager.addParameter(&custom_mqtt_ha_name);
#endif

  wifiManager.setTimeout(300);

  int counter = 0;

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("Thermometer", ""))
  {
    digitalWrite(pinAlarm, LOW);
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  while (WiFi.status() != WL_CONNECTED && counter != 10) {
    delay(500);
    Serial.print(".");
    display.clear();
    display.drawString(64, 10, "Connecting to WiFi");
    display.drawXbm(46, 30, 8, 8, counter % 3 == 0 ? activeSymbole : inactiveSymbole);
    display.drawXbm(60, 30, 8, 8, counter % 3 == 1 ? activeSymbole : inactiveSymbole);
    display.drawXbm(74, 30, 8, 8, counter % 3 == 2 ? activeSymbole : inactiveSymbole);
    display.display();

    counter++;
  }

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(workgroup, custom_workgroup.getValue());
  strcpy(username, custom_mqtt_user.getValue());
  strcpy(password, custom_mqtt_pass.getValue());
  strcpy(temp_scale, custom_temperature_scale.getValue());
#ifdef HOME_ASSISTANT_DISCOVERY
  strcpy(ha_name, custom_mqtt_ha_name.getValue());
#endif

  ArduinoOTA.onStart(otaStarted);
  ArduinoOTA.onEnd(otaFinished);
  ArduinoOTA.onProgress(otaProgress);
  ArduinoOTA.onError(otaError);
  ArduinoOTA.begin();

  Serial.print("MQTT Server: ");
  Serial.println(mqtt_server);
  Serial.print("MQTT Port: ");
  Serial.println(mqtt_port);
  // Print MQTT Username
  Serial.print("MQTT Username: ");
  Serial.println(username);
  // Hide password from the log and show * instead
  char hiddenpass[20] = "";
  for (size_t charP = 0; charP < strlen(password); charP++)
  {
    hiddenpass[charP] = '*';
  }
  hiddenpass[strlen(password)] = '\0';
  Serial.print("MQTT Password: ");
  Serial.println(hiddenpass);
  Serial.print("Saved temperature scale: ");
  Serial.println(temp_scale);
  configTempCelsius = ( (0 == strlen(temp_scale)) || String(temp_scale).equalsIgnoreCase("celsius"));
  Serial.print("Temperature scale: ");
  if (true == configTempCelsius)
  {
    Serial.println("Celsius");
  }
  else
  {
    Serial.println("Fahrenheit");
  }
#ifdef HOME_ASSISTANT_DISCOVERY
  Serial.print("Home Assistant sensor name: ");
  Serial.println(ha_name);
#endif

  const int mqttPort = atoi(mqtt_port);
  mqttClient.setServer(mqtt_server, mqttPort);
  mqttClient.setCallback(mqttCallback);

  mqttReconnect();

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveConfig();
  }

  // Get time from network time service
  configTime(TZ_SEC, DST_SEC, "pool.ntp.org");

  ui.setTargetFPS(30);

  ui.setActiveSymbol(activeSymbole);
  ui.setInactiveSymbol(inactiveSymbole);

  ui.setTimePerFrame(30 * 1000); // 10 sec

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(BOTTOM);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_TOP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  ui.setFrames(frames, numberOfFrames);

  ui.setOverlays(overlays, numberOfOverlays);

  // Inital UI takes care of initalising the display too.
  ui.init();

  Serial.println("");

  updateData(&display);

}

void loop() {
  ArduinoOTA.handle();

  // put your main code here, to run repeatedly:
  mqttClient.loop();

  if (millis() - timeSinceLastWUpdate > (1000L * UPDATE_INTERVAL_SECS)) {
    setReadyForWeatherUpdate();
    timeSinceLastWUpdate = millis();
  }

  if (readyForWeatherUpdate && ui.getUiState()->frameState == FIXED) {
    updateData(&display);
  }

  int remainingTimeBudget = ui.update();

  if (remainingTimeBudget > 0) {
    // Reconnect if there is an issue with the MQTT connection
    const unsigned long mqttConnectionMillis = millis();
    if ( (false == mqttClient.connected()) && (mqttConnectionInterval <= (mqttConnectionMillis - mqttConnectionPreviousMillis)) )
    {
      mqttConnectionPreviousMillis = mqttConnectionMillis;
      mqttReconnect();
    }

    const unsigned long currentMillis = millis();
    if (sensorInterval <= (currentMillis - sensorPreviousMillis))
    {
      sensorPreviousMillis = currentMillis;

      readSensors();

      if (!isnan(dhtTemperature) && !isnan(dhtHumidity))
      {
        publishSensorData("air/temperature", "temperature", convertTemperature(dhtTemperature));
        publishSensorData("air/humidity", "humidity", dhtHumidity);

        // Calculate heat index
        float dhtHeatIndex = dht.computeHeatIndex(dhtTemperature, dhtHumidity, false);
        publishSensorData("air/heatindex", "heatindex", convertTemperature(dhtHeatIndex));
      }

      publishSensorData("wifi/ssid", "ssid", WiFi.SSID());
      publishSensorData("wifi/ip", "ip", WiFi.localIP().toString());
    }

    delay(remainingTimeBudget);
  }

  // Press and hold the button to reset to factory defaults
  factoryReset();
}

void drawProgress(OLEDDisplay * display, int percentage, String label) {
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  display->drawString(64, 10, label);
  display->drawProgressBar(2, 28, 124, 10, percentage);
  display->display();
}

void updateData(OLEDDisplay * display) {
  drawProgress(display, 10, "Updating time...");
  drawProgress(display, 30, "Updating weather...");
  currentWeatherClient.setMetric(IS_METRIC);
  currentWeatherClient.setLanguage(OPEN_WEATHER_MAP_LANGUAGE);
  currentWeatherClient.updateCurrentById(&currentWeather, OPEN_WEATHER_MAP_APP_ID, OPEN_WEATHER_MAP_LOCATION_ID);
  drawProgress(display, 50, "Updating forecasts...");
  forecastClient.setMetric(IS_METRIC);
  forecastClient.setLanguage(OPEN_WEATHER_MAP_LANGUAGE);
  uint8_t allowedHours[] = {12};
  forecastClient.setAllowedHours(allowedHours, sizeof(allowedHours));
  forecastClient.updateForecastsById(forecasts, OPEN_WEATHER_MAP_APP_ID, OPEN_WEATHER_MAP_LOCATION_ID, MAX_FORECASTS);
  drawProgress(display, 80, "Reading sensors...");
  readSensors();
  readyForWeatherUpdate = false;
  drawProgress(display, 100, "Done...");
  delay(1000);
}

void drawDateTime(OLEDDisplay * display, OLEDDisplayUiState * state, int16_t x, int16_t y) {
  now = time(nullptr);
  struct tm* timeInfo;
  timeInfo = localtime(&now);
  char buff[16];

  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  String date = WDAY_NAMES[timeInfo->tm_wday];

  sprintf_P(buff, PSTR("%s, %02d/%02d/%04d"), WDAY_NAMES[timeInfo->tm_wday].c_str(), timeInfo->tm_mday, timeInfo->tm_mon + 1, timeInfo->tm_year + 1900);
  display->drawString(64 + x, 5 + y, String(buff));
  display->setFont(ArialMT_Plain_24);

  sprintf_P(buff, PSTR("%02d:%02d:%02d"), timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
  display->drawString(64 + x, 15 + y, String(buff));
  display->setTextAlignment(TEXT_ALIGN_LEFT);
}

void drawHomeCond(OLEDDisplay * display, OLEDDisplayUiState * state, int16_t x, int16_t y) {
  char buff[16];

  display->setFont(ArialMT_Plain_10);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(32 + x, 5 + y, "Temperature");

  display->setFont(ArialMT_Plain_16);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(32 + x, 18 + y, String(dhtTemperature, 1) + "C");

  display->setFont(ArialMT_Plain_10);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(96 + x, 5 + y, "Humidity");

  display->setFont(ArialMT_Plain_16);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(96 + x, 18 + y, String(dhtHumidity, 1) + "%");
}

void drawCurrentWeather(OLEDDisplay * display, OLEDDisplayUiState * state, int16_t x, int16_t y) {
  display->setFont(ArialMT_Plain_10);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(64 + x, 38 + y, currentWeather.description);

  display->setFont(ArialMT_Plain_24);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  String temp = String(currentWeather.temp, 1) + (IS_METRIC ? "°C" : "°F");
  display->drawString(60 + x, 5 + y, temp);

  display->setFont(Meteocons_Plain_36);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(32 + x, 0 + y, currentWeather.iconMeteoCon);
}

void drawForecast(OLEDDisplay * display, OLEDDisplayUiState * state, int16_t x, int16_t y) {
  drawForecastDetails(display, x, y, 0);
  drawForecastDetails(display, x + 44, y, 1);
  drawForecastDetails(display, x + 88, y, 2);
}

void drawForecastDetails(OLEDDisplay * display, int x, int y, int dayIndex) {
  time_t observationTimestamp = forecasts[dayIndex].observationTime;
  struct tm* timeInfo;
  timeInfo = localtime(&observationTimestamp);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  display->drawString(x + 20, y, WDAY_NAMES[timeInfo->tm_wday]);

  display->setFont(Meteocons_Plain_21);
  display->drawString(x + 20, y + 12, forecasts[dayIndex].iconMeteoCon);
  String temp = String(forecasts[dayIndex].temp, 0) + (IS_METRIC ? "°C" : "°F");
  display->setFont(ArialMT_Plain_10);
  display->drawString(x + 20, y + 34, temp);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
}

void drawHeaderOverlay(OLEDDisplay * display, OLEDDisplayUiState * state) {
  now = time(nullptr);
  struct tm* timeInfo;
  timeInfo = localtime(&now);
  char buff[14];
  sprintf_P(buff, PSTR("%02d:%02d"), timeInfo->tm_hour, timeInfo->tm_min);

  display->setColor(WHITE);
  display->setFont(ArialMT_Plain_10);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0, 54, String(buff));
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  String temp = String(dhtTemperature, 1) + (IS_METRIC ? "°C" : "°F");
  display->drawString(128, 54, temp);
  display->drawHorizontalLine(0, 52, 128);
}

void readSensors() {
  dhtTemperature = dht.readTemperature();
  dhtHumidity = dht.readHumidity();

  dhtTemperature = dhtTemperature * temperatureCoef;
}

void setReadyForWeatherUpdate() {
  Serial.println("Setting readyForUpdate to true");
  readyForWeatherUpdate = true;
}

void otaStarted() {
  Serial.println("OTA update started!");
}

void otaFinished() {
  Serial.println("OTA update finished!");
}

void otaProgress(unsigned int progress, unsigned int total) {
  Serial.printf("OTA progress: %u%%\r", (progress / (total / 100)));
}

void otaError(ota_error_t error) {
  Serial.printf("Error [%u]: ", error);
  switch (error)
  {
    case OTA_AUTH_ERROR:
      Serial.println("auth failed!");
      break;
    case OTA_BEGIN_ERROR:
      Serial.println("begin failed!");
      break;
    case OTA_CONNECT_ERROR:
      Serial.println("connect failed!");
      break;
    case OTA_RECEIVE_ERROR:
      Serial.println("receive failed!");
      break;
    case OTA_END_ERROR:
      Serial.println("end failed!");
      break;
  }
}

void factoryReset() {
  if (false == digitalRead(pinButton))
  {
    Serial.println("Hold the button to reset to factory defaults...");
    bool cancel = false;
    for (int iter = 0; iter < 30; iter++)
    {
      digitalWrite(pinAlarm, HIGH);
      delay(100);
      if (true == digitalRead(pinButton))
      {
        cancel = true;
        break;
      }
      digitalWrite(pinAlarm, LOW);
      delay(100);
      if (true == digitalRead(pinButton))
      {
        cancel = true;
        break;
      }
    }
    if (false == digitalRead(pinButton) && !cancel)
    {
      digitalWrite(pinAlarm, HIGH);
      Serial.println("Disconnecting...");
      WiFi.disconnect();

      // NOTE: the boot mode:(1,7) problem is known and only happens at the first restart after serial flashing.

      Serial.println("Restarting...");
      // Clean the file system with configurations
      SPIFFS.format();
      // Restart the board
      ESP.restart();
    }
    else
    {
      // Cancel reset to factory defaults
      Serial.println("Reset to factory defaults cancelled.");
      digitalWrite(pinAlarm, LOW);
    }
  }
}

void waitForFactoryReset() {
  Serial.println("Press button within 2 seconds for factory reset...");
  for (int iter = 0; iter < 20; iter++)
  {
    digitalWrite(pinAlarm, HIGH);
    delay(50);
    if (false == digitalRead(pinButton))
    {
      factoryReset();
      return;
    }
    digitalWrite(pinAlarm, LOW);
    delay(50);
    if (false == digitalRead(pinButton))
    {
      factoryReset();
      return;
    }
  }
}

void mqttCallback(char* topic, byte * payload, unsigned int length) {
  // Convert received bytes to a string
  char text[length + 1];
  snprintf(text, length + 1, "%s", payload);

  if (strcmp(topic, cmnd_temp_format) == 0)
  {
    processMessageScale(text);
  }

  publishState();
}

void calculateMachineId() {
  MD5Builder md5;
  md5.begin();
  char chipId[25];
  sprintf(chipId, "%d", ESP.getChipId());
  md5.add(chipId);
  md5.calculate();
  md5.toString().toCharArray(machineId, sizeof(machineId));
}

void mqttReconnect() {
  char clientId[18 + sizeof(machineId)];
  snprintf(clientId, sizeof(clientId), "anavi-thermometer-%s", machineId);

  // Loop until we're reconnected
  for (int attempt = 0; attempt < 3; ++attempt)
  {
    Serial.print("Connecting ... ");
    // Attempt to connect
    if (true == mqttClient.connect(clientId, username, password))
    {
      Serial.print("Connected");
      Serial.println(clientId);
      // Subscribe to MQTT topics
      mqttClient.subscribe(cmnd_temp_format);

      publishState();
      break;
    }
    else
    {
      Serial.print("Not connected");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishSensorData(const char* subTopic, const char* key, const float value) {
  StaticJsonDocument<100> json;
  json[key] = value;
  char payload[100];
  serializeJson(json, payload);
  char topic[200];
  sprintf(topic, "%s/%s/%s", workgroup, machineId, subTopic);
  mqttClient.publish(topic, payload, true);
  Serial.println("Publishing...");
  Serial.print("topic:");
  Serial.print(topic);
  Serial.print("payload:");
  Serial.println(payload);
}

void publishSensorData(const char* subTopic, const char* key, const String & value) {
  StaticJsonDocument<100> json;
  json[key] = value;
  char payload[100];
  serializeJson(json, payload);
  char topic[200];
  sprintf(topic, "%s/%s/%s", workgroup, machineId, subTopic);
  mqttClient.publish(topic, payload, true);
  Serial.println("Publishing...");
  Serial.print("topic:");
  Serial.print(topic);
  Serial.print("payload:");
  Serial.println(payload);
}

//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void saveConfig() {
  Serial.println("Saving configurations to file.");
  DynamicJsonDocument json(1024);
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["workgroup"] = workgroup;
  json["username"] = username;
  json["password"] = password;
  json["temp_scale"] = temp_scale;
#ifdef HOME_ASSISTANT_DISCOVERY
  json["ha_name"] = ha_name;
#endif

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("ERROR: failed to open config file for writing");
    return;
  }

  serializeJson(json, Serial);
  serializeJson(json, configFile);
  configFile.close();
}

void processMessageScale(const char* text) {
  StaticJsonDocument<200> data;
  deserializeJson(data, text);
  // Set temperature to Celsius or Fahrenheit and redraw screen
  Serial.print("Changing the temperature scale to: ");
  if (data.containsKey("scale") && (0 == strcmp(data["scale"], "celsius")) )
  {
    Serial.println("Celsius");
    configTempCelsius = true;
    strcpy(temp_scale, "celsius");
  }
  else
  {
    Serial.println("Fahrenheit");
    configTempCelsius = false;
    strcpy(temp_scale, "fahrenheit");
  }
  // Force default sensor lines with the new format for temperature

  // Save configurations to file
  saveConfig();
}

#ifdef HOME_ASSISTANT_DISCOVERY
bool publishSensorDiscovery(const char *config_key, const char *device_class, const char *name_suffix, 
                            const char *state_topic, const char *unit, const char *value_template) {
  static char topic[48 + sizeof(machineId)];

  snprintf(topic, sizeof(topic),
           "homeassistant/sensor/%s/%s/config", machineId, config_key);

  DynamicJsonDocument json(1024);
  json["device_class"] = device_class;
  json["name"] = String(ha_name) + " " + name_suffix;
  json["unique_id"] = String("anavi-") + machineId + "-" + config_key;
  json["state_topic"] = String(workgroup) + "/" + machineId + state_topic;
  json["unit_of_measurement"] = unit;
  json["value_template"] = value_template;

  json["device"]["identifiers"] = machineId;
  json["device"]["manufacturer"] = "ANAVI Technology";
  json["device"]["model"] = "ANAVI Thermometer";
  json["device"]["name"] = ha_name;
  json["device"]["sw_version"] = ESP.getSketchMD5();

  JsonArray connections = json["device"].createNestedArray("connections").createNestedArray();
  connections.add("mac");
  connections.add(WiFi.macAddress());

  Serial.print("Home Assistant discovery topic: ");
  Serial.println(topic);

  int payload_len = measureJson(json);
  if (!mqttClient.beginPublish(topic, payload_len, true))
  {
    Serial.println("beginPublish failed!\n");
    return false;
  }

  if (serializeJson(json, mqttClient) != payload_len)
  {
    Serial.println("writing payload: wrong size!\n");
    return false;
  }

  if (!mqttClient.endPublish())
  {
    Serial.println("endPublish failed!\n");
    return false;
  }

  return true;
}
#endif

void publishState() {
  static char payload[80];

#ifdef HOME_ASSISTANT_DISCOVERY
  String homeAssistantTempScale = (true == configTempCelsius) ? "°C" : "°F";
  publishSensorDiscovery("temp",
                         "temperature",
                         "Temperature",
                         "/air/temperature",
                         homeAssistantTempScale.c_str(),
                         "{{ value_json.temperature | round(1) }}");

  publishSensorDiscovery("humidity",
                         "humidity",
                         "Humidity",
                         "/air/humidity",
                         "%",
                         "{{ value_json.humidity | round(0) }}");

#endif
}

float convertCelsiusToFahrenheit(float temperature) {
  return (temperature * 9 / 5 + 32);
}

float convertTemperature(float temperature) {
  return (true == configTempCelsius) ? temperature : convertCelsiusToFahrenheit(temperature);
}

String formatTemperature(float temperature) {
  String unit = (true == configTempCelsius) ? "°C" : "°F";
  return String(convertTemperature(temperature), 1) + unit;
}

void setMQTTtopics() {
  // Set MQTT topics
  sprintf(cmnd_temp_format, "cmnd/%s/tempformat", machineId);
}
