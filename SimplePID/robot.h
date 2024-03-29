#include <GyverOLED.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

String ssid = "OK";
String password = "opencode24";

GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;

int LEDC_BASE_FREQ = 5000;
int LEDC_TIMER_12_BIT = 12;

SemaphoreHandle_t sensorValueMutex = NULL;

void sensors_update_task(void *);
void oled_task(void *);
void OTA();
void OTASetup(String, String);

struct Sensor
{
    int pin;
    int value;
    int maxValue;
    int minValue;
    int normalValue;
};

Sensor sens[4] = {{A0, 0, 4096, 0, 0}, {A4, 0, 4096, 0, 0}, {A5, 0, 4096, 0, 0}, {A3, 0, 4096, 0, 0}};

class Robot
{
public:
    Robot()
    {}

    void init(String OTASsid, String OTApwd, int buttonPin, int buttonPinMode = INPUT_PULLUP)
    {
        Serial.begin(115200);
        setButtonPin(buttonPin, buttonPinMode);
        oled.init();
        oled.setScale(1);
        if(isButtonPressed())
        {
            Serial.println("OTA RUNNING");
            oled.setScale(2);
            oled.setCursor(0, 0);
            oled.println("OTA RUNNING");
            oled.update();
            OTASetup(OTASsid, OTApwd);
            OTA();
        }
        sensorValueMutex = xSemaphoreCreateMutex();
        xTaskCreate(sensors_update_task, "Sensors update", 2048, NULL, 1, NULL);
    }

    void setSensPins(int sens1Pin, int sens2Pin, int sens3Pin, int sens4Pin)
    {
        sens[0].pin = sens1Pin;
        sens[1].pin = sens2Pin;
        sens[2].pin = sens3Pin;
        sens[3].pin = sens4Pin;

        pinMode(sens1Pin, INPUT_PULLDOWN);
        pinMode(sens2Pin, INPUT_PULLDOWN);
        pinMode(sens3Pin, INPUT_PULLDOWN);
        pinMode(sens4Pin, INPUT_PULLDOWN);
    }

    void setButtonPin(int buttonPin, int buttonPinMode = INPUT_PULLUP)
    {
        this->buttonPin = buttonPin;
        this->buttonPinMode = buttonPinMode;
        pressedState = buttonPinMode == INPUT_PULLUP ? LOW : HIGH;
        pinMode(buttonPin, buttonPinMode);
    }

    void setMotorsPins(int LeftA, int LeftB, int RightA, int RightB)
    {
        this->LeftA = LeftA;
        this->LeftB = LeftB;
        this->RightA = RightA;
        this->RightB = RightB;

        ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
        ledcAttachPin(LeftA, 0);
        ledcSetup(1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
        ledcAttachPin(LeftB, 1);
        ledcSetup(2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
        ledcAttachPin(RightA, 2);
        ledcSetup(3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
        ledcAttachPin(RightB, 3);
    }

    void move(int leftPower, int rightPower)
    {
        motor(0, 1, leftPower);
        motor(2, 3, rightPower);
    }

    void motor(int ch1, int ch2, int power)
    {
        if (power > 0)
        {
            ledcWrite(ch1, 0);
            ledcWrite(ch2, abs(power));
        }
        else
        {
            ledcWrite(ch1, abs(power));
            ledcWrite(ch2, 0);
        }
    }

    void mAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255)
    {
        // calculate duty, 4095 from 2 ^ 12 - 1
        uint32_t duty = (4095 / valueMax) * min(value, valueMax);

        // write duty to LEDC
        ledcWrite(channel, duty);
    }

    void stop()
    {
        motor(0, 1, 0);
        motor(2, 3, 0);
    }

    Sensor sensor(int num)
    {  
        // while (xSemaphoreTake(sensorValueMutex, portMAX_DELAY) != pdTRUE)
        // ;
        Sensor s = sens[num];
        // xSemaphoreGive(sensorValueMutex); 
        return s;
    }

    bool isButtonPressed()
    {
        if (digitalRead(buttonPin) == pressedState)
        {
            delay(50);
            return digitalRead(buttonPin) == pressedState;
        }
        return false;
    }

    bool isButtonClicked()
    {
        if (buttonBePressed){
            if (!isButtonPressed()) {
                buttonBePressed = false;
                return true;
            }
            else
                return false;
        }
        else if (isButtonPressed())
        {
            buttonBePressed = true;
            return false;
        }
        else {
            return false;
        }
        
    }

    void setSens(int num, int minValue, int maxValue, int normalValue)
    {
        sens[num].maxValue = maxValue;
        sens[num].minValue = minValue;
        sens[num].normalValue = normalValue;
    }

    void start()
    {
        xTaskCreatePinnedToCore(oled_task, "Oled update", 2048, NULL, 5, NULL, 0);
    }


private:
    int LeftA = 17;
    int LeftB = 16;
    int RightA = 19;
    int RightB = 18;

    int sens1Pin = A4;
    int sens2Pin = A5;
    int sens3Pin = A7;
    int sens4Pin = A6;

    int buttonPin = 15;
    int buttonPinMode = INPUT_PULLUP;
    int pressedState = LOW;
    int buttonBePressed = false;
};

int updateSensorValue(int sensPin, int *shareSensorValue)
{
    int measureSum = 0;
    for (int i = 0; i < 10; i++)
    {
        digitalWrite(14, HIGH);
        int value = 4096 - analogRead(sensPin);
        measureSum += value;
        // Serial.println(String(sensPin) + ": " + String(i) + ": " + String(value));
        digitalWrite(14, LOW);
        vTaskDelay(1);
    }
    // Serial.println();

    *shareSensorValue = measureSum / 10;

    return *shareSensorValue;
}

void sensors_update_task(void *pvParameters)
{
    while (1)
    {
        if (sensorValueMutex != NULL)
        { // Sanity check if the mutex exists
            // Try to take the mutex and wait indefintly if needed

            if (xSemaphoreTake(sensorValueMutex, portMAX_DELAY) == pdTRUE)
            {
                for (int sens_i = 0; sens_i < 4; sens_i++)
                {
                    sens[sens_i].value = updateSensorValue(sens[sens_i].pin, &sens[sens_i].value);
                }
                xSemaphoreGive(sensorValueMutex);

                vTaskDelay(1);
            }
        }
        else
        {
            vTaskDelay(1000);
        }
    }
}

void oled_task(void *pvParameters)
{
    while (1)
    {
        if (sensorValueMutex != NULL)
        {
            if (xSemaphoreTake(sensorValueMutex, portMAX_DELAY) == pdTRUE)
            {
                int sensorValue1 = sens[0].value;
                int sensorValue2 = sens[1].value;
                int sensorValue3 = sens[2].value;
                int sensorValue4 = sens[3].value;

                xSemaphoreGive(sensorValueMutex);
                oled.clear();             // очищаем экран
                oled.setCursor(0, 0);     // устанавливаем курсор в 0
                oled.print("s1: ");       // выводим значение
                oled.print(sensorValue1); // выводим значение
                oled.setCursor(64, 0);
                oled.print("s2: ");       // выводим значение
                oled.print(sensorValue2); // выводим значение
                oled.setCursor(0, 1);
                oled.print("s3: ");       // выводим значение
                oled.print(sensorValue3); // выводим значение
                oled.setCursor(64, 1);
                oled.print("s4: ");       // выводим значение
                oled.print(sensorValue4); // выводим значение
                oled.update();            // обновляем экран
                vTaskDelay(200);
            }
        }
        else
        {
            vTaskDelay(10);
        }
    }
}

void OTASetup(String OTASsid, String OTApwd)
{
  ssid = OTASsid;
  password = OTApwd;

  Serial.println("Booting");
  if (!WiFi.softAP(ssid.c_str(), password.c_str())) {
    log_e("Soft AP creation failed.");
    while(1);
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
   ArduinoOTA.setHostname("esp32");

  // No authentication by default
   ArduinoOTA.setPassword(password.c_str());

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void OTA(){
    while(1){
        ArduinoOTA.handle();
    }
}