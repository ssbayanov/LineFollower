#include "robot.h"

bool isEnabled = false;

Robot robot;

void calibrate()
{
    oled.setCursor(0, 0);
    oled.println("Calibrate sens1 min:");
    oled.update();

    int sensMinValue;
    while (!robot.isButtonClicked())
    {
        sensMinValue = robot.sensor(0).value;
        oled.setCursor(0, 1);
        oled.print("sens1: " + String(sensMinValue));
        oled.print(" ");
        oled.update();
        delay(200);
    }

    oled.setCursor(0, 0);
    oled.println("Calibrate sens1 max:");
    oled.update();

    oled.setCursor(0, 1);
    int sensMaxValue;
    while (!robot.isButtonClicked())
    {
        sensMaxValue = robot.sensor(0).value;
        oled.setCursor(0, 1);
        oled.print("sens1: " + String(sensMaxValue));
        oled.print(" ");
        oled.update();
        delay(200);
    }
    robot.setSens(0, sensMinValue, sensMaxValue, (sensMinValue + sensMaxValue) / 2);
}

void setup()
{
    robot.init("Ъ", "opencode24", 23, INPUT_PULLDOWN);
    // инициализация экрана
    oled.setCursor(0, 3);
    oled.println("Calibration...");
    oled.update();
    robot.setMotorsPins(16, 17, 18, 19);
    robot.setSensPins(A4, A5, A6, A7);

    calibrate();

    robot.start();
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

int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;

ulong prevTime = 0;

void loop()
{
    if (robot.isButtonClicked())
    {
        isEnabled = !isEnabled;
    }

    if (isEnabled)
    {

        // if (robot.sensor(0).value < robot.sensor(0).normalValue)
        // {
        //     robot.move(2048, 0);
        // }
        // else
        // {
        //     robot.move(0, 2048);
        // }

        ulong time = millis();
        sensorValue2 = robot.sensor(0).value;
        // sensorValue1 = shareSensorValue1;
        // sensorValue3 = shareSensorValue3;
        // sensorValue4 = shareSensorValue4;

        //                (вход,  установка, п, и, д, период в секундах, мин.выход, макс. выход)
        int value = computePID(sensorValue1, robot.sensor(0).normalValue, 6.5, 6, 1, (time - prevTime) / 1000.0, -4096, 4096);
        robot.move((float)(2048 - value / 2) * (1.0 - (abs(constrain(integral, -100, 100) / 200.0))),
                   (float)(2048 + value / 2) * (1.0 - (abs(constrain(integral, -100, 100) / 200.0))));

        // Serial.println("time: " + String((time - prevTime)));
        // Serial.println("PID value: " + String(value));
        // Serial.println("integral: " + String(integral));
        // Serial.println("Left power: " + String((1.0 - (abs(integral) / 8192.0))));
        prevTime = time;
    }
    else
    {
        robot.stop();
    }
}