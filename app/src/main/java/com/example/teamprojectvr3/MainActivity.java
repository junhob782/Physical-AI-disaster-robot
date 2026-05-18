package com.example.teamprojectvr3;

import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import androidx.appcompat.app.AppCompatActivity;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "RobotApp";

    // UI 컴포넌트 변수 선언
    private EditText etIp, etPort;
    private Button btnConnect, btnEmergency;
    private TextView tvStatus, tvGas, tvTemp, tvHumidity;

    // 탭 제어 관련 변수
    private Button btnTabDrive, btnTabArm;
    private View layoutDriveMap, layoutArmCamera;
    private View layoutControlDrive, layoutControlArm;

    // 로봇팔 조작 위젯 변수
    private SeekBar sbJoint1, sbJoint2;
    private Button btnGripperOpen, btnGripperClose;
    private int currentJoint1 = 90;
    private int currentJoint2 = 90;

    // 순정 커스텀 조이스틱 위젯 변수
    private View viewJoystickBg, viewJoystickStick;
    private float centerX, centerY, jbRadius;
    private long lastSendTime = 0;

    // 로봇 물리 한계 속도 셋팅값
    private static final double MAX_LINEAR_X = 0.5;   // 최대 전진 속도 (m/s)
    private static final double MAX_ANGULAR_Z = 1.0;  // 최대 회전 속도 (rad/s)

    // [핵심] 네트워킹 및 팀원 C의 자동 재연결 로직 변수 통합
    private Socket socket;
    private PrintWriter writer;
    private BufferedReader reader;
    private volatile boolean isRunning = false; // 재연결 루프 제어 플래그
    private boolean isConnected = false;        // 현재 실질적 연결 상태 식별용
    private Thread connectionThread = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // 1. 기본 제어 및 설정 위젯 연결
        etIp = findViewById(R.id.et_ip);
        etPort = findViewById(R.id.et_port);
        btnConnect = findViewById(R.id.btn_connect);
        btnEmergency = findViewById(R.id.btn_emergency);
        tvStatus = findViewById(R.id.tv_status);

        tvGas = findViewById(R.id.tv_sensor_gas);
        tvTemp = findViewById(R.id.tv_sensor_temp);
        tvHumidity = findViewById(R.id.tv_sensor_humidity);

        // 2. 주행 / 로봇팔 탭 전환 위젯 연결
        btnTabDrive = findViewById(R.id.btn_tab_drive);
        btnTabArm = findViewById(R.id.btn_tab_arm);
        layoutDriveMap = findViewById(R.id.layout_drive_map);
        layoutArmCamera = findViewById(R.id.layout_arm_camera);
        layoutControlDrive = findViewById(R.id.layout_control_drive);
        layoutControlArm = findViewById(R.id.layout_control_arm);

        // 3. 로봇팔 슬라이더 및 제어 컴포넌트 연결
        sbJoint1 = findViewById(R.id.sb_joint1);
        sbJoint2 = findViewById(R.id.sb_joint2);
        btnGripperOpen = findViewById(R.id.btn_gripper_open);
        btnGripperClose = findViewById(R.id.btn_gripper_close);

        // 순정 조이스틱 객체 바인딩
        viewJoystickBg = findViewById(R.id.view_joystick_bg);
        viewJoystickStick = findViewById(R.id.view_joystick_stick);

        // 4. 로봇 서버 연결 시작 / 수동 종료 버튼 리스너
        btnConnect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (!isRunning) {
                    String ip = etIp.getText().toString().trim();
                    String portStr = etPort.getText().toString().trim();

                    if (ip.isEmpty() || portStr.isEmpty()) {
                        Toast.makeText(MainActivity.this, "IP와 포트를 입력하세요.", Toast.LENGTH_SHORT).show();
                        return;
                    }

                    int port = Integer.parseInt(portStr);

                    // 팀원 C의 엔진 시동: 자동 재연결 플래그 ON 및 루프 작동
                    isRunning = true;
                    btnConnect.setText("중지");
                    startAutoConnectionLoop(ip, port);
                } else {
                    // 사용자가 명시적으로 중지 버튼을 누르면 루프 완전 종료 및 세션 단절
                    stopAutoConnectionLoop();
                }
            }
        });

        // 5. 최우선순위 비상정지 버튼 리스너
        btnEmergency.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sendEmergencyStop();
            }
        });

        // 6. 탭 스위칭 리스너 (주행)
        btnTabDrive.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                btnTabDrive.setBackgroundTintList(android.content.res.ColorStateList.valueOf(Color.parseColor("#333333")));
                btnTabDrive.setTextColor(Color.WHITE);
                btnTabArm.setBackgroundTintList(android.content.res.ColorStateList.valueOf(Color.parseColor("#222222")));
                btnTabArm.setTextColor(Color.parseColor("#888888"));

                layoutDriveMap.setVisibility(View.VISIBLE);
                layoutControlDrive.setVisibility(View.VISIBLE);
                layoutArmCamera.setVisibility(View.GONE);
                layoutControlArm.setVisibility(View.GONE);
            }
        });

        // 7. 탭 스위칭 리스너 (로봇팔)
        btnTabArm.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                btnTabArm.setBackgroundTintList(android.content.res.ColorStateList.valueOf(Color.parseColor("#333333")));
                btnTabArm.setTextColor(Color.WHITE);
                btnTabDrive.setBackgroundTintList(android.content.res.ColorStateList.valueOf(Color.parseColor("#222222")));
                btnTabDrive.setTextColor(Color.parseColor("#888888"));

                layoutDriveMap.setVisibility(View.GONE);
                layoutControlDrive.setVisibility(View.GONE);
                layoutArmCamera.setVisibility(View.VISIBLE);
                layoutControlArm.setVisibility(View.VISIBLE);
            }
        });

        // 8. 순정 안드로이드 터치 이벤트 활용 알고리즘 -> ROS 2 속도 벡터 수식 변환 엔진
        layoutControlDrive.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (centerX == 0) {
                    centerX = viewJoystickBg.getX() + (viewJoystickBg.getWidth() / 2f);
                    centerY = viewJoystickBg.getY() + (viewJoystickBg.getHeight() / 2f);
                    jbRadius = viewJoystickBg.getWidth() / 2f;
                }

                float touchX = event.getX();
                float touchY = event.getY();

                if (event.getAction() == MotionEvent.ACTION_MOVE || event.getAction() == MotionEvent.ACTION_DOWN) {
                    float displacement = (float) Math.sqrt(Math.pow(touchX - centerX, 2) + Math.pow(touchY - centerY, 2));

                    double angleRad = Math.atan2(centerY - touchY, touchX - centerX);
                    double ratio = Math.min(1.0, displacement / jbRadius);

                    // 조이스틱 좌표계를 ROS 2 원격 기하 제어 매커니즘(cmd_vel) 속도로 변환
                    double linearX = Math.sin(angleRad) * ratio * MAX_LINEAR_X;
                    double angularZ = Math.cos(angleRad) * ratio * MAX_ANGULAR_Z;

                    linearX = Math.round(linearX * 1000.0) / 1000.0;
                    angularZ = Math.round(angularZ * 1000.0) / 1000.0;

                    if (displacement < jbRadius) {
                        viewJoystickStick.setX(touchX - (viewJoystickStick.getWidth() / 2f));
                        viewJoystickStick.setY(touchY - (viewJoystickStick.getHeight() / 2f));
                    } else {
                        float constraintRatio = jbRadius / displacement;
                        float constrainedX = centerX + (touchX - centerX) * constraintRatio;
                        float constrainedY = centerY + (touchY - centerY) * constraintRatio;
                        viewJoystickStick.setX(constrainedX - (viewJoystickStick.getWidth() / 2f));
                        viewJoystickStick.setY(constrainedY - (viewJoystickStick.getHeight() / 2f));
                    }

                    long currentTime = System.currentTimeMillis();
                    if (currentTime - lastSendTime >= 100) {
                        lastSendTime = currentTime;
                        sendDriveVelocity(linearX, angularZ);
                    }

                } else if (event.getAction() == MotionEvent.ACTION_UP) {
                    viewJoystickStick.setX(centerX - (viewJoystickStick.getWidth() / 2f));
                    viewJoystickStick.setY(centerY - (viewJoystickStick.getHeight() / 2f));
                    sendDriveVelocity(0.0, 0.0);
                }
                return true;
            }
        });

        // 9. 로봇팔 관절 제어 리스너 설정
        sbJoint1.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if(fromUser) {
                    currentJoint1 = progress;
                    sendArmCommand(currentJoint1, currentJoint2, -1);
                }
            }
            @Override public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override public void onStopTrackingTouch(SeekBar seekBar) {}
        });

        sbJoint2.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if(fromUser) {
                    currentJoint2 = progress;
                    sendArmCommand(currentJoint1, currentJoint2, -1);
                }
            }
            @Override public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override public void onStopTrackingTouch(SeekBar seekBar) {}
        });

        btnGripperOpen.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sendArmCommand(currentJoint1, currentJoint2, 1);
            }
        });

        btnGripperClose.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sendArmCommand(currentJoint1, currentJoint2, 0);
            }
        });
    }

    // [팀원 C 로직 고도화 통합] 2초 주기 자동 재연결 매커니즘 스레드 오케스트레이션
    private void startAutoConnectionLoop(final String ip, final int port) {
        connectionThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (isRunning) {
                    try {
                        updateStatusUi("상태: 연결 시도 중...", Color.YELLOW);
                        Log.d(TAG, "로봇 브릿지 서버 연결 시도 중... IP: " + ip + ", PORT: " + port);

                        socket = new Socket(ip, port);
                        writer = new PrintWriter(socket.getOutputStream(), true);
                        reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));

                        isConnected = true;
                        updateStatusUi("상태: 연결됨", Color.GREEN);
                        showToastOnUi("로봇 브릿지 서버와 관제 세션이 수립되었습니다.");

                        // 연결이 정상 유지되는 동안에는 이 내부 루프에 머물며 1초마다 소켓 단절을 감시 (자원 낭비 방지)
                        while (isRunning && isConnected && !socket.isClosed()) {
                            Thread.sleep(1000);

                            // 인바운드 스트림 상태 체크를 위한 수신 파트 수동 유기 결합
                            startReceivingAndRouting();
                        }

                    } catch (Exception e) {
                        isConnected = false;
                        writer = null;
                        reader = null;

                        updateStatusUi("상태: 연결 유실 (재시도 중)", Color.RED);
                        Log.e(TAG, "통신 채널 단절 감지! 2초 후 자동 재연결을 시도합니다: " + e.getMessage());

                        try {
                            Thread.sleep(2000); // 2초(2000ms) 휴식 후 다시 while의 처음으로 돌아가 재연결 수행
                        } catch (InterruptedException ie) {
                            Log.e(TAG, "재연결 대기 인터럽트 예외 트리거");
                        }
                    }
                }
            }
        });
        connectionThread.start();
    }

    // [서버 수신 파트] 실시간 인바운드 다중 JSON 파싱 라우터 (자동 재연결 커널 결합 버전)
    private void startReceivingAndRouting() {
        try {
            String line;
            // 스트림에 대기 중인 패킷 라인이 있을 때만 루프 수행 (블로킹 누수 방지)
            while (reader != null && (line = reader.readLine()) != null) {
                final String message = line;
                Log.d(TAG, "서버 수신 원본 데이터: " + message);

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            org.json.JSONObject jsonObject = new org.json.JSONObject(message);
                            if (!jsonObject.has("msg_type")) return;

                            String msgType = jsonObject.getString("msg_type");

                            if (msgType.equals("sensor_data")) {
                                int gas = jsonObject.getInt("gas_level");
                                double temp = jsonObject.getDouble("temperature");
                                double humidity = jsonObject.getDouble("humidity");

                                tvGas.setText("가스: " + gas + " ppm");
                                tvTemp.setText("온도: " + temp + " °C");
                                tvHumidity.setText("습도: " + humidity + " %");

                            } else if (msgType.equals("survivor_alert")) {
                                int heartRate = jsonObject.getInt("heart_rate");
                                double locX = jsonObject.getDouble("location_x");
                                double locY = jsonObject.getDouble("location_y");

                                String alertText = "🚨 생존자 탐지!! 심박수: " + heartRate + " bpm (X:" + locX + ", Y:" + locY + ")";
                                Toast.makeText(MainActivity.this, alertText, Toast.LENGTH_LONG).show();
                                Log.w(TAG, "⚠️ [ALERT] 구조 대상자 발견 -> " + alertText);
                            }
                        } catch (Exception e) {
                            Log.e(TAG, "인바운드 JSON 파싱 예외: " + e.getMessage());
                        }
                    }
                });
            }
        } catch (IOException e) {
            // 소켓 파이프 파손 시 내부 루프 탈출을 유도하여 재연결 블록으로 넘김
            isConnected = false;
        }
    }

    // [송신 1] 주행 조작 패킷 빌더
    private void sendDriveVelocity(final double linearX, final double angularZ) {
        Log.d(TAG, "🟢 [주행 제어 연산] -> linear_x: " + linearX + ", angular_z: " + angularZ);

        if (isConnected && writer != null) {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        org.json.JSONObject driveJson = new org.json.JSONObject();
                        driveJson.put("msg_type", "cmd_vel");
                        driveJson.put("linear_x", linearX);
                        driveJson.put("angular_z", angularZ);

                        writer.println(driveJson.toString());
                    } catch (Exception e) {
                        Log.e(TAG, "주행 데이터 패킷 유실: " + e.getMessage());
                    }
                }
            }).start();
        }
    }

    // [송신 2] 로봇팔 조작 패킷 빌더
    private void sendArmCommand(final int j1, final int j2, final int gripperState) {
        final int finalGripper = (gripperState == -1) ? 1 : gripperState;
        Log.d(TAG, "🤖 [로봇팔 제어 연산] -> joint1: " + j1 + ", joint2: " + j2 + ", gripper: " + finalGripper);

        if (isConnected && writer != null) {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        org.json.JSONObject armJson = new org.json.JSONObject();
                        armJson.put("msg_type", "arm_cmd");
                        armJson.put("joint1", j1);
                        armJson.put("joint2", j2);
                        armJson.put("gripper", finalGripper);

                        writer.println(armJson.toString());
                    } catch (Exception e) {
                        Log.e(TAG, "로봇팔 데이터 패킷 유실: " + e.getMessage());
                    }
                }
            }).start();
        }
    }

    // [송신 3] 비상정지 패킷 빌더
    private void sendEmergencyStop() {
        Log.d(TAG, "🚨 [비상 정지 명령 생성 완료]");
        if (isConnected && writer != null) {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        org.json.JSONObject emergencyJson = new org.json.JSONObject();
                        emergencyJson.put("msg_type", "emergency_stop");

                        writer.println(emergencyJson.toString());

                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                Toast.makeText(MainActivity.this, "🚨 원격 로봇 비상정지 명령 전송!", Toast.LENGTH_SHORT).show();
                            }
                        });
                    } catch (Exception e) {
                        Log.e(TAG, "비상정지 송신 에러: " + e.getMessage());
                    }
                }
            }).start();
        } else {
            Toast.makeText(this, "서버 연결 단절 상태입니다.", Toast.LENGTH_SHORT).show();
        }
    }

    // 사용자가 직접 관제를 중단하려 할 때 루프 완전 수동 파쇄
    private void stopAutoConnectionLoop() {
        isRunning = false;
        isConnected = false;
        btnConnect.setText("연결");

        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    if (writer != null) writer.close();
                    if (reader != null) reader.close();
                    if (socket != null) socket.close();
                    Log.d(TAG, "네트워크 커널 자원 완전 폐쇄 성공");
                } catch (IOException e) {
                    Log.e(TAG, "세션 종료 컨텍스트 해제 실패: " + e.getMessage());
                }
            }
        }).start();

        updateStatusUi("상태: 끊김", Color.RED);
        Toast.makeText(this, "원격 관제 연결 엔진이 명시적으로 중단되었습니다.", Toast.LENGTH_SHORT).show();
    }

    // UI 안전 스레드 갱신 래퍼용 헬퍼 메서드들
    private void updateStatusUi(final String statusText, final int color) {
        runOnUiThread(() -> {
            tvStatus.setText(statusText);
            tvStatus.setTextColor(color);
        });
    }

    private void showToastOnUi(final String msg) {
        runOnUiThread(() -> Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show());
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopAutoConnectionLoop();
    }
}