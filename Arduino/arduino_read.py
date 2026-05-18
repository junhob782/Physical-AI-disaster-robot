import serial
import json
import time

arduino_port = 'COM7'
baud_rate = 115200

try:
    # 시리얼 통신 연결
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # 아두이노 리셋 후 안정화될 때까지 2초 대기
    print(f"Connected to Arduino! ({arduino_port}) - Starting data...\n")

    while True:
        if ser.in_waiting > 0:
            # 1. 아두이노에서 날아온 JSON 문자열 한 줄 읽기
            raw_data = ser.readline().decode('utf-8').rstrip()

            try:
                # 2. 텍스트를 파이썬 딕셔너리로 변환 (파싱)
                sensor_data = json.loads(raw_data)

                distance = sensor_data["distance"]
                gas = sensor_data["gas"]
                temp = sensor_data["temperature"]
                hum = sensor_data["humidity"]

                print(f"📡 Received | distance: {distance}cm | gas: {gas} | temp: {temp}℃ | hum: {hum}%")

                # ==========================================
                # 🚨 무인 경계 감시 시스템 핵심 로직 판단 부분 🚨
                # ==========================================

                # 침입자 감지 (예: 50cm 이내로 뭔가 다가왔을 때)
                if distance < 50.0:
                    print("  [Robot] Robot sensor measurement")

                # 화재/유해가스 감지 (가스 수치가 평소보다 확 튀었을 때)
                if gas > 200:
                    print("  [DANGER] Harmful gas level rising!!")

                # ==========================================

            except json.JSONDecodeError:
                # 가끔 데이터가 끊겨서 들어오면 무시하고 다음 데이터 받음
                pass

except serial.SerialException:
    print(f"port {arduino_port} Failed.")
except KeyboardInterrupt:
    print("\nSurveillance system terminated.")
    if 'ser' in locals() and ser.is_open:
        ser.close()
