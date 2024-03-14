int sensor = A6;

void setup() {
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(16, 0);
  ledcAttachPin(17, 1);

  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(18, 2);
  ledcAttachPin(19, 3);

  pinMode(sensor, INPUT_PULLUP);

  Serial.begin(115200);
}

void motor(int ch1, int ch2, int power) // power: -255 - 255
{
  if (power > 0) {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, abs(power));
  }
  else
  {
    ledcWrite(ch1, abs(power));
    ledcWrite(ch2, 0);
  }
}

void loop() {
  int sensorValue = analogRead(sensor);

  Serial.println(sensorValue);

  if (sensorValue > 2200) {
    motor(0, 1, 255);
    motor(2, 3, 0);
  }
  else
  {
    motor(0, 1, 0);
    motor(2, 3, 255);
  }
  delay(30);
}
