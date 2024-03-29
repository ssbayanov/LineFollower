#include <GyverOLED.h>                       // Подключение библиотеки для работы с экранос
GyverOLED<SSD1306_128x32, OLED_BUFFER> oled; // объявление переменной для работы с экраном

// переменные с пинами моторов
int LeftA = 17;
int LeftB = 16;

int RightA = 19;
int RightB = 18;

const int sens1Pin = A4;
const int sens2Pin = A5;
const int sens3Pin = A6;
const int sens4Pin = A7;

int shareSensorValue1 = 0;
int shareSensorValue2 = 0;
int shareSensorValue3 = 0;
int shareSensorValue4 = 0;
SemaphoreHandle_t sensorValueMutex = NULL;

// параметры работы ШИМ
#define LEDC_TIMER_12_BIT 12 // 12 разрядов на установку значений (0 - 4095)
#define LEDC_BASE_FREQ 5000  // частота ШИМ модуляции

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

    sensorValueMutex = xSemaphoreCreateMutex();

    static int task_number0 = 0;
    xTaskCreate(
        robot_move_task, "Move task" // A name just for humans
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

    static int task_number1 = 1;
    xTaskCreate(
        oled_task, "OLED task", 2048 // Stack size
        ,
        (void *)&task_number1 // Pass reference to a variable describing the task number
        ,
        1 // Low priority
        ,
        NULL // Task handle is not used here - simply pass NULL
    );
}

// функция цикла выполняется снова и снова, бесконечно:
void loop()
{
}

int updateSensorValue(int sensPin, int *shareSensorValue){
    *shareSensorValue = analogRead(sensPin);
    return *shareSensorValue;
}

void robot_move_task(void *pvParameters)
{
    int sensorValue1 = 0;
    int sensorValue2 = 0;
    int sensorValue3 = 0;
    int sensorValue4 = 0;
    while (1)
    {
        if (sensorValueMutex != NULL)
        { // Sanity check if the mutex exists
            // Try to take the mutex and wait indefintly if needed

            if (xSemaphoreTake(sensorValueMutex, portMAX_DELAY) == pdTRUE)
            {
                sensorValue1 = updateSensorValue(sens1Pin, &shareSensorValue1);
                sensorValue2 = updateSensorValue(sens2Pin, &shareSensorValue2);
                sensorValue3 = updateSensorValue(sens3Pin, &shareSensorValue3);
                sensorValue4 = updateSensorValue(sens4Pin, &shareSensorValue4);

                // shareSensorValue1 = analogRead(sens1Pin);
                // sensorValue1 = shareSensorValue1;
                // shareSensorValue2 = analogRead(sens2Pin);
                // sensorValue2 = shareSensorValue2;

                xSemaphoreGive(sensorValueMutex);

                if (sensorValue1 < 200)
                {
                    move(0, 1, 0);
                    move(2, 3, 255);
                }
                else
                {
                    move(0, 1, 255);
                    move(2, 3, 0);
                }
            }

            vTaskDelay(10);
        }
        else
        {
            vTaskDelay(10);
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
                oled.clear();            // очищаем экран
                oled.setCursor(0, 0);    // устанавливаем курсор в 0
                oled.print("s1: ");   // выводим значение
                oled.print(sensorValue1); // выводим значение
                oled.setCursor(64, 0);
                oled.print("s2: ");   // выводим значение
                oled.print(sensorValue2); // выводим значение
                oled.setCursor(0, 1);
                oled.print("s3: ");   // выводим значение
                oled.print(sensorValue3); // выводим значение
                oled.setCursor(64, 1);
                oled.print("s4: ");   // выводим значение
                oled.print(sensorValue4); // выводим значение
                oled.update();           // обновляем экран
                vTaskDelay(200);
            }
            else
            {
                vTaskDelay(10);
            }
        }
        else
        {
            vTaskDelay(10);
        }
    }
}