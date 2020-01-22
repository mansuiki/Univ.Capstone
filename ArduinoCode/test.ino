#include <Wire.h>

#define  vibe1 9      //중지
#define  vibe2 10     //검지

int v1 = 0;
int v2 = 0;
int skip = 0;
int vibeMode = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(vibe1, OUTPUT);
  pinMode(vibe2, OUTPUT);
  analogWrite(vibe1, 255);
  analogWrite(vibe2, 255);
  delay(1000);
  analogWrite(vibe1, v1);
  analogWrite(vibe2, v2);
}

void loop() {
  
  if (Serial.available() > 0) {
    vibeMode = Serial.read();
  } else {
    vibeMode = 0;
  }
  vibe(vibeMode);
  analogWrite(vibe1, v1);
  analogWrite(vibe2, v2);
  Serial.print(Serial.available());
  Serial.print("\n");
  delay(50);
}

void vibe(int vibeMode) {
  switch (vibeMode) {
    case 0:
      v1 = 0;
      v2 = 0;
      break;
    case 48:     //1단계
      v1 = 0;
      v2 = 0;
      break;
    case 49:     //2단계
      v1 = 0;
      v2 = 255;
      break;
    case 50:     //3단계
      v1 = 255;
      v2 = 255;
      break;

    default:
      break;
  }
}
