#include <GyverOLED.h> // Подключение библиотеки для работы с экранос
GyverOLED<SSD1306_128x32, OLED_BUFFER> oled; // объявление переменной для работы с экраном

// переменные с пинами моторов
int LeftA = 25;
int LeftB = 26;

int RightA = 32;
int RightB = 33;

// параметры работы ШИМ
#define LEDC_TIMER_12_BIT  12 // 12 разрядов на установку значений (0 - 4095)
#define LEDC_BASE_FREQ     5000 // частота ШИМ модуляции


// функция установки значения шим с ограничением по верхнему значению
void mAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}


// функция установки мощности мотора 0 - 255
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
  // инициализация вывода в последовательный порт
  Serial.begin(115200);

  // инициализация экрана
  oled.init();
  oled.setScale(2);


  // инициализация шим по порту
  ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(LeftA, 0);
  ledcSetup(1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(LeftB, 1);
  ledcSetup(2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(RightA, 2);
  ledcSetup(3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(RightB, 3);

}

// функция цикла выполняется снова и снова, бесконечно:
void loop() {
  // считываем значение с датчика в переменную
  int sensorValue = analogRead(A6);
  
  
  oled.clear(); // очищаем экран  
  oled.setCursor(0, 0); // устанавливаем курсор в 0
  oled.print(sensorValue); // выводим значение  
  oled.update(); // обновляем экран

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

  //Serial.println(sensorValue); // вывод значения
  delay(10);        // delay in between reads for stability
}
