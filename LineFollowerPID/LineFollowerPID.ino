#include <GyverOLED.h>                       // Подключение библиотеки для работы с экранос
GyverOLED<SSD1306_128x32, OLED_BUFFER> oled; // объявление переменной для работы с экраном


int freq = 50;

// переменные с пинами моторов
int LeftA = 17;
int LeftB = 16;

int RightA = 19;
int RightB = 18;

const int sens1Pin = A4;
const int sens2Pin = A5;
const int sens3Pin = A7;
const int sens4Pin = A6;

int shareSensorValue1 = 0;
int shareSensorValue2 = 0;
int shareSensorValue3 = 0;
int shareSensorValue4 = 0;
SemaphoreHandle_t sensorValueMutex = NULL;

bool moveEnabled = false;

// параметры работы ШИМ
#define LEDC_TIMER_12_BIT 12 // 12 разрядов на установку значений (0 - 4095)
#define LEDC_BASE_FREQ 10000  // частота ШИМ модуляции

// функция установки значения шим с ограничением по верхнему значению
void mAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255)
{
    // calculate duty, 4095 from 2 ^ 12 - 1
    uint32_t duty = (4095 / valueMax) * min(value, valueMax);

    // write duty to LEDC
    ledcWrite(channel, duty);
}

// функция установки мощности мотора 0 - 255
void move(int a, int b, int power)
{
    if (power > 0)
    {
        mAnalogWrite(a, power, 255);
        mAnalogWrite(b, 0, 255);
    }
    else
    {
        mAnalogWrite(a, 0, 255);
        mAnalogWrite(b, power, 255);
    }
}

void robot_move_task(void *pvParameters);
void oled_task(void *pvParameters);

// the setup routine runs once when you press reset:
void setup()
{
    // инициализация вывода в последовательный порт
    Serial.begin(115200);

    // инициализация экрана
    oled.init();
    oled.setScale(1);

    // инициализация шим по порту
    ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
    ledcAttachPin(LeftA, 0);
    ledcSetup(1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
    ledcAttachPin(LeftB, 1);
    ledcSetup(2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
    ledcAttachPin(RightA, 2);
    ledcSetup(3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
    ledcAttachPin(RightB, 3);
    // pinMode(LeftA, OUTPUT);
    // pinMode(LeftB, OUTPUT);
    // pinMode(RightA, OUTPUT);
    // pinMode(RightB, OUTPUT);

    sensorValueMutex = xSemaphoreCreateMutex();

    pinMode(sens1Pin, INPUT_PULLDOWN);
    pinMode(sens2Pin, INPUT_PULLDOWN);
    pinMode(sens3Pin, INPUT_PULLDOWN);
    pinMode(sens4Pin, INPUT_PULLDOWN);

    pinMode(14, OUTPUT);

    pinMode(23, INPUT_PULLUP);

    static int task_number0 = 0;
    xTaskCreate(
        sensors_update_task, "Sensors" // A name just for humans
        ,
        2048 // The stack size
        ,
        (void *)&task_number0 // Pass reference to a variable describing the task number
        ,
        5 // High priority
        //,  1  // priority
        ,
        NULL // Task handle is not used here - simply pass NULL
    );

    static int task_number2 = 2;
    xTaskCreate(
        oled_task, "OLED task", 2048 // Stack size
        ,
        (void *)&task_number2 // Pass reference to a variable describing the task number
        ,
        1 // Low priority
        ,
        NULL // Task handle is not used here - simply pass NULL
    );
}

// функция цикла выполняется снова и снова, бесконечно:
void loop()
{
    // robot_move_task(NULL);
    if (digitalRead(23) == LOW)
    {
        delay(50);
        if (digitalRead(23) == LOW)
        {
            while (digitalRead(23) != HIGH)
                ;
            moveEnabled = !moveEnabled;
            if (moveEnabled)
            {
                static int task_number1 = 1;
                xTaskCreate(
                    robot_move_task, "Move task", // A name just for humans
                    2048,                         // The stack size
                    (void *)&task_number1,        // Pass reference to a variable describing the task number
                    5,                            // High priority
                    NULL                          // Task handle is not used here - simply pass NULL
                );
            }
            else
            {
                vTaskDelete(xTaskGetHandle("Move task"));
                move(0, 1, 0);
                move(2, 3, 0);
            }
        }
    }
}

int updateSensorValue(int sensPin, int *shareSensorValue)
{
    int measureSum = 0;
    for (int i = 0; i < 10; i++)
    {   
        digitalWrite(14, HIGH);
        int value = 4096 - analogRead(sensPin);
        measureSum += value;
        Serial.println(String(sensPin) + ": " + String(i) + ": " + String(value));
        digitalWrite(14, LOW);
        vTaskDelay(1);
    }
    Serial.println();

    *shareSensorValue = measureSum / 10;

    return *shareSensorValue;
}


float integral = 0, prevErr = 0;
// функция пид
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut)
{
    float err = setpoint - input;
    integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
    float D = (err - prevErr) / dt;
    prevErr = err;
    return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

// float pidCalculation(float sensorValue, float setpoint, float kp, float ki, float kd, float prevError, float integral, float dt) {
//     float error = setpoint - sensorValue;
//     integral = constrain(integral + error * dt * ki, -100, 100);
//     float D = (error - prevError) / dt;
//     prevError = error;
//     return constrain(error * kp + integral + D * kd, -100, 100);
// }

void sensors_update_task(void *pvParameters)
{
    while (1)
    {
        if (sensorValueMutex != NULL)
        { // Sanity check if the mutex exists
            // Try to take the mutex and wait indefintly if needed

            if (xSemaphoreTake(sensorValueMutex, portMAX_DELAY) == pdTRUE)
            {
                updateSensorValue(sens1Pin, &shareSensorValue1);
                updateSensorValue(sens2Pin, &shareSensorValue2);
                updateSensorValue(sens3Pin, &shareSensorValue3);
                updateSensorValue(sens4Pin, &shareSensorValue4);

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

void robot_move_task(void *pvParameters)
{
    int sensorValue1 = 0;
    int sensorValue2 = 0;
    int sensorValue3 = 0;
    int sensorValue4 = 0;

    ulong prevTime = 0;
    integral = 0;
    prevErr = 0;

    while (1)
    {
        if (sensorValueMutex != NULL)
        { // Sanity check if the mutex exists
            // Try to take the mutex and wait indefintly if needed

            if (xSemaphoreTake(sensorValueMutex, portMAX_DELAY) == pdTRUE)
            {
                ulong time = millis();
                sensorValue2 = shareSensorValue2;
                sensorValue1 = shareSensorValue1;
                sensorValue3 = shareSensorValue3;
                sensorValue4 = shareSensorValue4;

                xSemaphoreGive(sensorValueMutex);

                //                (вход,  установка, п, и, д, период в секундах, мин.выход, макс. выход)
                int value = computePID(sensorValue1, 760, 6.5, 6, 1, (time - prevTime) / 1000.0, -4096, 4096);
                move(0, 1, (float)(160 - value / 16) * (1.0 - (abs(constrain(integral, -100, 100) / 200.0))));
                move(2, 3, (float)(160 + value / 16) * (1.0 - (abs(integral) / 8192.0)));

                // Serial.println("time: " + String((time - prevTime)));
                // Serial.println("PID value: " + String(value));
                // Serial.println("integral: " + String(integral));
                // Serial.println("Left power: " + String((1.0 - (abs(integral) / 8192.0))));
                prevTime = time;
                // delay(2000);
                vTaskDelay(20);
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
                int sensorValue1 = shareSensorValue1;
                int sensorValue2 = shareSensorValue2;
                int sensorValue3 = shareSensorValue3;
                int sensorValue4 = shareSensorValue4;

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