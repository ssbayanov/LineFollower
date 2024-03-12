#include <GyverOLED.h>
GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;

int LeftA = 25;
int LeftB = 26;

int RightA = 32;
int RightB = 33;

#define LEDC_TIMER_12_BIT  12
#define LEDC_BASE_FREQ     5000

void mAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void move(int a, int b, int power)
{
  if (power > 0) {
    mAnalogWrite(a, power, 255);
    mAnalogWrite(b, 0, 255);
  }
  else {
    mAnalogWrite(a, 0, 255);
    mAnalogWrite(b, power, 255);
  }
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  oled.init();
  oled.setScale(2);

  ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(LeftA, 0);
  ledcSetup(1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(LeftB, 1);
  ledcSetup(2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(RightA, 2);
  ledcSetup(3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(RightB, 3);

//  move(0, 1, 255);
//  move(2, 3, 255);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A6);
  // print out the value you read:
  oled.clear();
  oled.setCursor(0, 0);
  oled.print(sensorValue);
  oled.update(); 

  if( sensorValue < 200)
  {
    move(0, 1, 0);
    move(2, 3, 255);
  }
  else
  {
    move(0, 1, 255);
    move(2, 3, 0);
  }

  //Serial.println(sensorValue);
  delay(10);        // delay in between reads for stability
}
