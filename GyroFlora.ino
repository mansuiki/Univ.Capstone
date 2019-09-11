#include <Wire.h>
#include <TimerOne.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define  vibe1 9
#define  vibe2 10

const int LLL   = 7500;
const int LL    = 5400;
const int LC    = 2400;
const int CR    = -2500;
const int RR    = -6000;
const int RRR   = -8000;

const int FFF   = 8000;
const int FF    = 5200;
const int FC    = 800;
const int CB    = -2800;
const int BB    = -6000;
const int BBB   = -7500;

int state[3];
int vibeMode = 0;
long int ti;
long int cpt = 0;
volatile bool intFlag = false;


void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(vibe1, OUTPUT);
  pinMode(vibe2, OUTPUT);

  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);

  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}

void callback()
{
  intFlag = true;
}

void loop() {
  vibeMode = Serial.Read();
  vibe();
  
  while (!intFlag);
  intFlag = false;

  //=====Read accelerometer and gyroscope=====
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  // Accelerometer
  int16_t ax = -(Buf[0] << 8 | Buf[1]);
  int16_t ay = -(Buf[2] << 8 | Buf[3]);
  int16_t az = Buf[4] << 8 | Buf[5];
  // Gyroscope
  int16_t gx = -(Buf[8] << 8 | Buf[9]);
  int16_t gy = -(Buf[10] << 8 | Buf[11]);
  int16_t gz = Buf[12] << 8 | Buf[13];

  //=====Read magnetometer data=====
  uint8_t ST1;
  do  {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  }
  while (!(ST1 & 0x01));
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

  // Magnetometer
  int16_t mx = -(Mag[3] << 8 | Mag[2]);
  int16_t my = -(Mag[1] << 8 | Mag[0]);
  int16_t mz = -(Mag[5] << 8 | Mag[4]);

  //=====Display values=====
  // Accelerometer
  Serial.print (ax, DEC);   //손가락 앞뒤 회전 (중앙 :0, 앞~뒤: 8000 ~ -8000)
  Serial.print ("\t");
  Serial.print (ay, DEC);   //손 좌우 회전 (중앙:0, 좌~우: 8000 ~ -8000)
  Serial.print ("\t");
  Serial.print (az, DEC);   //손 좌우 회전(최대: 중앙 8000, 좌우 기울일때마다 최소 -5000까지)
  Serial.print ("\t");

  // Gyroscope
  //  Serial.print (gx,DEC);    //좌우 기울임 가속도
  //  Serial.print ("\t");
  //  Serial.print (gy,DEC);    //부정확, 앞뒤 기울임 가속도
  //  Serial.print ("\t");
  //  Serial.print (gz,DEC);    //손목 좌우 회전 가속도
  //  Serial.print ("\t");

  // Magnetometer
  //  Serial.print (mx,DEC);    //부정확
  //  Serial.print ("\t");
  //  Serial.print (my,DEC);    //좌우 기울임 (중앙:0, 좌~우: -400~400)
  //  Serial.print ("\t");
  //  Serial.print (mz, DEC);   //플로팅 값나옴
  //  Serial.print ("\t");

  Gyro(ax, ay, az);
  Serial.println("");
  delay(100);
}

void vibe(int vibeMode) {
  switch (vibeMode) {
    case 0:     //(0,0)단계
      analogWrite(vibe1, 0);
      analogWrite(vibe2, 0);
    case 1:     //(0,1)단계
      analogWrite(vibe1, 0);
      analogWrite(vibe2, 170);
    case 2:     //(0,2)단계
      analogWrite(vibe1, 0);
      analogWrite(vibe2, 255);
    case 3:     //(1,0)단계
      analogWrite(vibe1, 170);
      analogWrite(vibe2, 0);
    case 4:     //(1,1)단계
      analogWrite(vibe1, 170);
      analogWrite(vibe2, 170);
    case 5:     //(1,2)단계
      analogWrite(vibe1, 170);
      analogWrite(vibe2, 255);
    case 6:     //(2,0)단계
      analogWrite(vibe1, 255);
      analogWrite(vibe2, 0);
    case 7:     //(2,1)단계
      analogWrite(vibe1, 255);
      analogWrite(vibe2, 170);
    case 8:     //(2,2)단계
      analogWrite(vibe1, 255);
      analogWrite(vibe2, 170);
  }
}
int Gyro(int16_t ax, int16_t ay, int16_t az) {
  //state[손등방향 상0/하1, 전0~후6, 좌0~우6]
  //===========극좌표계 양의 축 기준 골무 방향=============
  //state[0] 상하
  //  (0)0~180 / (1)180~360
  //state[1] 전후, state[2] 좌우
  //  (0) 90~ 85 / (1) 85~ 45 / (2) 45~ 15 / (3) 15~-15
  //  (6)-90~-85 / (5)-85~-45 / (4)-45~-15
  //============예시============
  //손등이 연직상방일때: [0, 3, 3]
  //악수할때 (오른손): [0, 3, 6]

  if (az > 0) {
    state[0] = 0;
  } else {
    state[0] = 1;
  }

  if (ax > FFF) {
    state[1] = 0;
  } else if (ax > FF) {
    state[1] = 1;
  } else if (ax > FC) {
    state[1] = 2;
  } else if (ax > CB) {
    state[1] = 3;
  } else if (ax > BB) {
    state[1] = 4;
  } else if (ax > BBB) {
    state[1] = 5;
  } else {
    state[1] = 6;
  }

  if (ay > LLL) {
    state[2] = 0;
  } else if (ay > LL) {
    state[2] = 1;
  } else if (ay > LC) {
    state[2] = 2;
  } else if (ay > CR) {
    state[2] = 3;
  } else if (ay > RR) {
    state[2] = 4;
  } else if (ay > RRR) {
    state[2] = 5;
  } else {
    state[2] = 6;
  }

  Serial.print(state[0]);
  Serial.print(state[1]);
  Serial.print(state[2]);
}
