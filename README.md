# RESCUE-Link 시스템 기술 스택 (Tech Stack)

## 📱 1. 모바일 앱 (클라이언트 계층)
* **운영체제:** Android
* **개발 언어:** Java
  * **주요 라이브러리:** Gson (JSON 문자열을 Java 객체로 한 번에 변환)
* **개발 환경 (IDE):** Android Studio
* **핵심 기술:**
  * UI/UX 레이아웃 설계 (XML 기반)
  * 안드로이드 소켓(Socket) 통신 클라이언트 구축
  * 실시간 비디오 스트리밍 디코딩 및 UI 오버레이
  * JSON 데이터 파싱 및 대시보드 시각화

---

## 🧠 2. 백엔드 (두뇌 계층)
* **운영체제:** Ubuntu 22.04 LTS (Jetson Nano / 로봇 PC)
* **미들웨어:** ROS2 Humble
* **서버 개발 언어:** C/C++, Python
  * **TCP/IP 소켓 통신:** C++ (네트워크 지연 최소화 및 고성능 라우팅)
  * **YOLO26 모델 추론:** Python (AI 모델 로딩 및 실시간 데이터 처리)
* **비전 AI:** YOLO26, OpenCV
* **자율주행:** SLAM, Nav2 (내비게이션 스택)

---

## 🦾 3. 하드웨어 및 펌웨어 (물리 계층)
* **마이크로컨트롤러:** Arduino Mega 2560
* **개발 언어:** C++ (Arduino 펌웨어)
* **개발 환경 (IDE):** Arduino IDE
* **주요 센서 및 액추에이터:**
  * **주행:** 메카넘 휠
  * **로봇 팔:** 4DOF 서보 모터
  * **환경 인지:** LiDAR, Depth 카메라
  * **데이터 수집:** * MQ-2 (가스 농도)
    * DHT11 (온습도)
    * MPU6050 (자이로/가속도)
    * MAX30102 (심박수)
* **통신:** PC-Arduino 간 시리얼 통신 (UART) 및 JSON 직렬화
* **라이브러리 :** ArduinoJson


---

## 🛠️ 4. 공통 협업 및 인프라
* **버전 관리:** GitHub
* **데이터 포맷:** JSON (시스템 전반의 표준 메시지 규격)
* **네트워크/API 테스트:** Packet Sender
