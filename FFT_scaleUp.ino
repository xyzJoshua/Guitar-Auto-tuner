#include <Adafruit_NeoPixel.h>
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

/*
  These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
double signalFrequency = 1000;
double samplingFrequency = 5000;
uint8_t amplitude = 100; //8bit
#define PIN 2 // 아두이노 연결핀(컨트롤핀)
#define NUMPIXELS 8 // 네오픽셀 갯수
Adafruit_NeoPixel pixels =
  Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
/*
  These are the input and output vectors 입력과 출력 벡터
  Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];
int j = 64; // 512 = 360degree
int high, low;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

#define Theta 6.2831 //2Pi

#define DELAY_TIME 4// 지연시간 4[msec]

void setup()
{

  Serial.begin(115200);
  pinMode(12, OUTPUT); // A 상
  pinMode(11, OUTPUT); // B 상
  pinMode(10, OUTPUT); // ~A 상
  pinMode(9, OUTPUT); // ~B 상
  digitalWrite(12, LOW); // A상 구동 출력포트 Low 로 초기화
  digitalWrite(11, LOW); // B상 구동 출력포트 Low 로 초기화
  digitalWrite(10, LOW); // ~A상 구동 출력포트 Low 로 초기화
  digitalWrite(9, LOW); // ~B상 구동 출력포트 Low 로 초기화
}

void loop()
{
  for (uint8_t i = 0; i < samples; i++) //0.0128 s
  {
    vReal[i] = analogRead(A0);
    delayMicroseconds(100);
    vImag[i] = 0;
  }
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_RECTANGLE, FFT_REVERSE);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_REVERSE); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY); //samples >> 1 = 64
  
}

void PrintVector(double *vData, uint8_t bufferSize, uint8_t scaleType)
{
  high = 0, low = 0;
  for (uint16_t i = 2; i < bufferSize; i++)
  {
    uint8_t val_temp = map(vData[i], 0, 1000, 0, 255);
    print_value(i, val_temp);
    sensing(i, val_temp);
  }
  control();
}

void control() {
  if (high > 3) { //음이 높은 경우
    ledOn(0);//Red_LED
    twoPhaseCW();
  }
  else if (low > 1) { //음이 낮은 경우
    ledOn(2);//Blue_LED
    twoPhaseCCW();
  }
  else{
    ledOn(1);//Green_LED
    }
}

void sensing(uint16_t i, uint8_t val_temp) { //val_temp = 센서 데이터값 a = 5번 줄 데이터 값
  double a[] = {0, 0, 9, 0, 0, 6, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //5번 줄 정상값

  if (a[i] == 0) { //값이 0일 때
    if (a[i]+1 < val_temp) {
      Serial.println("High");
      high++;
    }
    else {
      Serial.println("Green");
    }
  }
  else { //값이 1이상일 때
    if (val_temp == 0) {
      Serial.println("Green");
    }
    else if ((a[i] - 0) > val_temp) {
      Serial.println("Low");
      low++;
    }
    else if ((a[i] + 2) < val_temp) { 
      Serial.println("High");
      high++;
    }
    else if (val_temp == 0){
      Serial.println("Green");
    }
  }
}

void print_value(uint16_t i, uint8_t val_temp) {
  Serial.print(val_temp);
  Serial.print(" ");
  Serial.println(i);
}

void twoPhaseCW() { //2상여자 시계방향 (느슨하게)
  // A,B상
  for (int i = 0; i <= j; i++) {
    digitalWrite(12, HIGH); // A,B상 ON
    digitalWrite(11, HIGH);
    delay(DELAY_TIME);  // DELAY_TIME 동안 A상 ON 유지
    digitalWrite(12, LOW); // A,B상 OFF
    digitalWrite(11, LOW);

    // ~A,B상
    digitalWrite(10, HIGH); // ~A,B상 ON
    digitalWrite(11, HIGH);
    delay(DELAY_TIME);  // DELAY_TIME 동안 ~B상 ON 유지
    digitalWrite(10, LOW); // ~A,B상 OFF
    digitalWrite(11, LOW); // ~A,B상 OFF

    // ~A,~B상
    digitalWrite(10, HIGH); // ~A,~B상 ON
    digitalWrite(9, HIGH);
    delay(DELAY_TIME);  // DELAY_TIME 동안 ~A상 ON 유지
    digitalWrite(10, LOW); // ~A,~B상 OFF
    digitalWrite(9, LOW);

    // A,~B상
    digitalWrite(12, HIGH); // A,~B상 ON
    digitalWrite(9, HIGH);
    delay(DELAY_TIME);  // DELAY_TIME 동안 B상 ON 유지
    digitalWrite(12, LOW); // A,~B상상 OFF
    digitalWrite(9, LOW);
  }
}

void twoPhaseCCW() { //2상여자 반시계방향 (팽팽하게)
  for (int i = 0; i <= j; i++) {
    // A,B상
    digitalWrite(12, HIGH); // A,B상 ON
    digitalWrite(11, HIGH);
    delay(DELAY_TIME);  // DELAY_TIME 동안 A상 ON 유지
    digitalWrite(12, LOW); // A,B상 OFF
    digitalWrite(11, LOW);

    // A,~B상
    digitalWrite(12, HIGH); // A,~B상 ON
    digitalWrite(9, HIGH);
    delay(DELAY_TIME);  // DELAY_TIME 동안 B상 ON 유지
    digitalWrite(12, LOW); // A,~B상상 OFF
    digitalWrite(9, LOW);

    // ~A,~B상
    digitalWrite(10, HIGH); // ~A,~B상 ON
    digitalWrite(9, HIGH);
    delay(DELAY_TIME);  // DELAY_TIME 동안 ~A상 ON 유지
    digitalWrite(10, LOW); // ~A,~B상 OFF
    digitalWrite(9, LOW);

    // ~A,B상
    digitalWrite(10, HIGH); // ~A,B상 ON
    digitalWrite(11, HIGH);
    delay(DELAY_TIME);  // DELAY_TIME 동안 ~B상 ON 유지
    digitalWrite(10, LOW); // ~A,B상 OFF
    digitalWrite(11, LOW); // ~A,B상 OFF
  }
}

void motorClean() {
  digitalWrite(12, LOW); // A상 구동 출력포트 Low 로 초기화
  digitalWrite(11, LOW); // B상 구동 출력포트 Low 로 초기화
  digitalWrite(10, LOW); // ~A상 구동 출력포트 Low 로 초기화
  digitalWrite(9, LOW); // ~B상 구동 출력포트 Low 로 초기화
}

void ledOn(int color){
  int red,green,blue;
  pixels.begin(); // neoPixel Start
  
  
  switch(color){
    case 0: // LED RED
      red = 255,green = 0, blue = 0;
      break;
    case 1: // LED GREEN
      red = 0,green = 255, blue = 0;
      break;
    case 2: // LED BLUE
      red = 0,green = 0, blue = 255;
      break;
    case 3: // LED 초기화
      red = 0,green = 0, blue = 0;
      break;
    }
  for(int i=0;i<=7;i++){
        pixels.setPixelColor(i,pixels.Color(red,green,blue));//green
        pixels.show();
      }
  
  }
