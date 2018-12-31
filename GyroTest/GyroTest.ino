
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

String fieldsName[] = {"Temp", "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "AccAngX", "AccAngeY", "GyroAngX", "GyroAngY", "GyroAngY", "AngleX", "AngleY", "AngleZ"};
float datas[15];
long timer = 0;
int i;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();

  if (millis() - timer > 1000) {
    i = 0;
    datas[i++] = mpu6050.getTemp();
    datas[i++] = mpu6050.getAccX();
    datas[i++] = mpu6050.getAccY();
    datas[i++] = mpu6050.getAccZ();

    datas[i++] = mpu6050.getGyroX();
    datas[i++] = mpu6050.getGyroY();
    datas[i++] = mpu6050.getGyroZ();

    datas[i++] = mpu6050.getAccAngleX();
    datas[i++] = mpu6050.getAccAngleY();

    datas[i++] = mpu6050.getGyroAngleX();
    datas[i++] = mpu6050.getGyroAngleY();
    datas[i++] = mpu6050.getGyroAngleZ();

    datas[i++] = mpu6050.getAngleX();
    datas[i++] = mpu6050.getAngleY();
    datas[i++] = mpu6050.getAngleZ();
/*
    Serial.println("=======================================================");
    String columnsList = "";
    for ( i = 0; i < 15; i++)
      Serial.print(fieldsName[i] + "\t");
    Serial.println("");
    */
    String data = "";
    for (i = 0; i < 15; i++)
      Serial.print(datas[i]); Serial.print("\t\t");
    Serial.println("");
    //Serial.println("=======================================================\n");
    timer = millis();

  }

}
