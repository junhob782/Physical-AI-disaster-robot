//아두이노 센서 코드 단순

#include <Wire.h>
#include "Adafruit_HTU21DF.h" 

// --- 핀 번호 설정 ---
// 1. 초음파 센서 핀
const int trigPin = 2;
const int echoPin = 3;

// 2. 가스 센서 핀
const int gasPin = A0;

// 3. 온습도 센서 (I2C) 객체 생성 (SDA: 20번, SCL: 21번 핀 사용)
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

// --- 타이머 변수 ---
unsigned long previousMillis = 0;
const long interval = 1000; // 1초마다 전송

void setup() {
  Serial.begin(115200);
  
  // 초음파 핀 초기화
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // I2C 온습도 센서 시작
  if (!htu.begin()) {
    Serial.println("온습도 센서 에러! 20번(SDA), 21번(SCL) 및 3.3V 전원 연결을 확인해주세요.");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // --- 1. 센서 데이터 읽기 ---
    
    // [초음파 센서] 거리 계산
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    float distance = duration * 0.034 / 2.0; 

    // [가스 센서]
    int gasValue = analogRead(gasPin);

    // [온습도 센서 I2C]
    float temperature = htu.readTemperature();
    float humidity = htu.readHumidity();

    // --- 2. JSON 문자열 포맷팅 ---
    // 예시: {"distance":150.25, "gas":312, "temperature":25.4, "humidity":45.2}
    String jsonString = "{";
    jsonString += "\"distance\":" + String(distance, 2) + ",";
    jsonString += "\"gas\":" + String(gasValue) + ",";
    jsonString += "\"temperature\":" + String(temperature, 1) + ",";
    jsonString += "\"humidity\":" + String(humidity, 1);
    jsonString += "}";

    // --- 3. 시리얼로 전송 ---
    Serial.println(jsonString);
  }
}
