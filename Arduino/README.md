Android 앱
    ↓
Ubuntu 서버 PC
    ├── 조이스틱 수신
    ├── Arduino 센서 읽기
    └── ROS2 publish
            ↓
ROS2 로봇 Ubuntu
            ↓
실제 바퀴임

지금 단계에서는 Arduino는 “센서값을 시리얼로 보내기만” 하면 됨.

가스센서: MQ 계열 (A0)
초음파센서: HC-SR04
온습도센서: DHT11
