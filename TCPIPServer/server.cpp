// =============================================================================
// rescue_bridge_server.cpp
// =============================================================================
// [한 줄 요약] 안드로이드 앱 ↔ ROS2 사이의 TCP/JSON 브릿지 서버
//
// [전체 흐름]
//   안드로이드 앱 ──(TCP + JSON)──> 이 서버 (수신·파싱·라우팅)
//                                  └──> 향후 ROS2 토픽 발행 [TODO 위치]
//                                  └──> SafetyMonitor 트리거 등
//
// [빌드 / 실행]
//   설치: sudo apt install nlohmann-json3-dev
//   빌드: colcon build --packages-select rescue_bridge
//   실행: ros2 run rescue_bridge server
//   종료: Ctrl+C            (안전 종료 — 시그널 처리됨)
//
// [통신 규약 — 안드로이드 앱 개발자가 반드시 알아야 할 것]
//   - 매 JSON 메시지 끝에 '\n' (개행) 필수
//   - 예) {"msg_type":"cmd_vel","linear_x":0.5,"angular_z":0.1}\n
//   - 서버 응답도 '\n' 으로 끝남
//   - 한 번에 여러 메시지 묶어서 보내도 OK (예: "msg1\nmsg2\nmsg3\n")
//
// [구조 — 클래스 4개]
//   Socket         : fd 자동 close (RAII 패턴)
//   LineReader     : TCP 스트림을 줄 단위 메시지로 분리
//   MessageRouter  : msg_type 별 핸들러 등록·실행
//   TcpServer      : 소켓 생성·대기·클라이언트 처리 루프
//
// [지원 메시지 타입]
//   - cmd_vel        : 주행 명령 (조이스틱)
//   - arm_cmd        : 로봇팔 관절 + 그리퍼
//   - emergency_stop : 비상정지
//   - nav_goal       : 자율주행 목표 좌표
//   - ping           : 연결 확인 (Watchdog)
// =============================================================================


// ─── 표준 라이브러리 ──────────────────────────────────
#include <iostream>           // std::cout
#include <string>             // std::string
#include <atomic>             // std::atomic — 시그널 핸들러와의 동기화
#include <csignal>            // std::signal, SIGINT, SIGTERM
#include <cstring>            // 시스템콜에서 사용
#include <functional>         // std::function — 람다를 핸들러로 저장
#include <optional>           // std::optional — "값 있음/없음" 표현
#include <unordered_map>      // 핸들러 등록용 해시맵
#include <cerrno>             // errno (시스템콜 오류 코드)

// ─── POSIX 소켓 API ───────────────────────────────────
#include <unistd.h>           // ::close()
#include <arpa/inet.h>        // sockaddr_in, htons, inet_ntop
#include <sys/socket.h>       // socket(), bind(), listen(), accept(), send(), recv()

// ─── 외부 라이브러리 ──────────────────────────────────
#include <nlohmann/json.hpp>  // JSON 파싱·생성 (단일 헤더 라이브러리)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// nlohmann::json 풀네임이 길어서 별칭 사용
using json = nlohmann::json;


// =============================================================================
// 전역 종료 플래그 (시그널 핸들러용)
// =============================================================================
namespace {
    std::atomic<bool> g_running{true};

    // SIGINT/SIGTERM 받으면 호출됨. 단순히 플래그만 false 로.
    void on_signal(int /*sig*/) {
        g_running = false;
    }
}


// =============================================================================
// 클래스 1: Socket — 파일 디스크립터 RAII 래퍼
// =============================================================================
class Socket {
    int fd_ = -1;  // -1 = "유효하지 않음" 표시

public:
    Socket() = default;
    explicit Socket(int fd) : fd_(fd) {}

    // 복사 금지
    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

    // 이동: fd 소유권만 옮김. 원래 객체는 -1 로 무효화.
    Socket(Socket&& o) noexcept : fd_(o.fd_) {
        o.fd_ = -1;
    }
    Socket& operator=(Socket&& o) noexcept {
        if (this != &o) {
            reset();          // 기존 fd 가 있으면 먼저 닫고
            fd_ = o.fd_;
            o.fd_ = -1;       // 원래 객체 무효화
        }
        return *this;
    }

    // 소멸자: 객체 수명 끝나면 자동 close
    ~Socket() { reset(); }

    int  fd()    const { return fd_; }
    bool valid() const { return fd_ >= 0; }

    // 명시적으로 닫고 싶을 때 (또는 소멸자에서 호출됨)
    void reset() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
};


// =============================================================================
// 클래스 2: LineReader — TCP 스트림을 줄 단위로 분리
// =============================================================================
class LineReader {
    int         fd_;     // 읽을 소켓의 fd
    std::string buf_;    // 누적 버퍼 (다음 메시지 일부가 미리 들어와 있을 수 있음)

public:
    explicit LineReader(int fd) : fd_(fd) {}

    std::optional<std::string> read_line() {
        char chunk[2048];   // recv() 한 번에 받을 임시 버퍼

        while (g_running) {
            // [1단계] 누적 버퍼에 이미 '\n' 이 있는지 검사
            if (auto pos = buf_.find('\n'); pos != std::string::npos) {
                std::string line = buf_.substr(0, pos);  // '\n' 앞까지 잘라내고
                buf_.erase(0, pos + 1);                  // 처리한 부분은 제거
                return line;
            }

            // [2단계] '\n' 없으면 더 받기
            ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);

            if (n > 0) {
                // 정상 수신 → 버퍼에 누적하고 다시 1단계로
                buf_.append(chunk, n);
                continue;
            }
            if (n == 0) {
                // 상대(peer)가 정상 종료 — FIN 패킷 받음
                return std::nullopt;
            }
            // n < 0 → 오류
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // SO_RCVTIMEO 로 설정한 1초 timeout → 종료 플래그 재확인을 위해
                continue;
            }
            return std::nullopt;  // 그 외 진짜 오류
        }
        return std::nullopt;  // g_running == false (종료 요청)
    }
};


// =============================================================================
// 클래스 3: MessageRouter — msg_type 별 핸들러 디스패치
// =============================================================================
class MessageRouter {
public:
    using Handler = std::function<json(const json&)>;

    void on(std::string type, Handler h) {
        handlers_[std::move(type)] = std::move(h);
    }

    json dispatch(const std::string& raw) const {
        try {
            json m = json::parse(raw);
            std::string type = m.value("msg_type", "");

            auto it = handlers_.find(type);
            if (it == handlers_.end()) {
                return err("unknown msg_type: '" + type + "'");
            }
            return it->second(m);
        }
        catch (const json::parse_error& e) {
            return err(std::string("invalid JSON: ") + e.what());
        } catch (const json::type_error& e) {
            return err(std::string("type error: ") + e.what());
        } catch (const json::out_of_range& e) {
            return err(std::string("missing field: ") + e.what());
        }
    }

private:
    static json err(std::string msg) {
        return {{"status", "error"}, {"message", std::move(msg)}};
    }

    std::unordered_map<std::string, Handler> handlers_;
};


// =============================================================================
// 클래스 4: TcpServer — 소켓 라이프사이클 관리
// =============================================================================
class TcpServer {
public:
    explicit TcpServer(int port) : port_(port) {}

    bool start() {
        int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            std::perror("socket");
            return false;
        }
        listen_ = Socket(fd);

        int opt = 1;
        ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        timeval tv{1, 0};   // 1초 0마이크로초
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(port_);

        if (::bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
            std::perror("bind");
            return false;
        }

        if (::listen(fd, 1) < 0) {
            std::perror("listen");
            return false;
        }

        std::cout << "[Server] port " << port_ << " LISTEN\n";
        return true;
    }

    void run(const MessageRouter& router) {
        while (g_running) {
            sockaddr_in caddr{};
            socklen_t   clen = sizeof(caddr);
            int cfd = ::accept(listen_.fd(), (sockaddr*)&caddr, &clen);

            if (cfd < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                if (!g_running) break;
                std::perror("accept");
                continue;
            }

            timeval tv{1, 0};
            ::setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            char ip[INET_ADDRSTRLEN];
            ::inet_ntop(AF_INET, &caddr.sin_addr, ip, sizeof(ip));
            std::cout << "[Server] client connected: " << ip << "\n";

            serve_client(Socket(cfd), router);

            std::cout << "[Server] client disconnected, waiting for next...\n";
        }
    }

private:
    void serve_client(Socket sock, const MessageRouter& router) {
        LineReader reader(sock.fd());

        while (g_running) {
            auto line = reader.read_line();
            if (!line) break;
            if (line->empty()) continue;

            json resp = router.dispatch(*line);
            std::string out = resp.dump() + "\n";

            ssize_t sent = ::send(sock.fd(), out.data(), out.size(), MSG_NOSIGNAL);
            if (sent <= 0) break;
        }
    }

    int    port_;
    Socket listen_;
};


// =============================================================================
// main — 핸들러 등록 + ROS2 통합 서버 실행
// =============================================================================
int main(int argc, char* argv[]) {
    // 1. 포트 결정 및 시그널 핸들러 등록
    int port = (argc > 1) ? std::stoi(argv[1]) : 8080;
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    // ─── ROS2 초기화 및 노드 생성 ──────────────────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rescue_bridge_node");
    
    // 로봇의 바퀴로 명령을 보낼 Publisher 생성 (/cmd_vel 토픽)
    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // TCP 서버(while 루프)가 블로킹되더라도 ROS2가 뒤에서 돌아갈 수 있도록 별도 스레드로 분리
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });

    MessageRouter router;

    // ─────────────────────────────────────────────
    // [cmd_vel] 로봇 주행 명령 수신 및 ROS2 퍼블리시
    // ─────────────────────────────────────────────
    router.on("cmd_vel", [cmd_vel_pub](const json& m) {
        double lx = m.at("linear_x").get<double>();
        double az = m.at("angular_z").get<double>();
        std::cout << "  [cmd_vel 수신] 전진: " << lx << " 회전: " << az << "\n";

        // JSON 속도 값을 ROS2 Twist 메시지로 변환하여 로봇으로 전송
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = lx;
        twist_msg.angular.z = az;
        cmd_vel_pub->publish(twist_msg);

        return json{{"status", "ok"}, {"echo", "cmd_vel"}};
    });

    // ─────────────────────────────────────────────
    // [emergency_stop] 비상정지 (최우선 처리)
    // ─────────────────────────────────────────────
    // 람다 캡처에 cmd_vel_pub을 추가하고, 속도를 0으로 쏴서 로봇을 강제 정지시킵니다.
    router.on("emergency_stop", [cmd_vel_pub](const json&) {
        std::cout << "  🚨 [E-STOP 수신] 긴급 비상정지! 로봇의 모든 동작을 정지합니다.\n";
        
        // 속도가 모두 0.0으로 세팅된 빈 Twist 메시지 생성
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        
        // ROS2 /cmd_vel 토픽으로 즉시 강제 발행!
        cmd_vel_pub->publish(stop_msg);

        return json{{"status", "ok"}, {"echo", "emergency_stop"}};
    });

    // ─────────────────────────────────────────────
    // [기타 핸들러] arm_cmd, nav_goal, ping
    // ─────────────────────────────────────────────
    router.on("arm_cmd", [](const json& m) {
        /* ... 향후 구현 예정 ... */
        return json{{"status", "ok"}, {"echo", "arm_cmd"}};
    });
    router.on("nav_goal", [](const json& m) {
        /* ... 향후 구현 예정 ... */
        return json{{"status", "ok"}, {"echo", "nav_goal"}};
    });
    router.on("ping", [](const json&) {
        return json{{"status", "ok"}, {"type", "pong"}};
    });

    // ─── 서버 실행 ─────────────────────────────────
    TcpServer server(port);
    if (!server.start()) {
        rclcpp::shutdown();
        return 1; 
    }
    
    std::cout << "[Server] ROS2 브릿지 가동! 조이스틱 명령을 대기합니다.\n";
    
    // 종료 시그널(Ctrl+C)을 받을 때까지 클라이언트(안드로이드) 응대
    server.run(router);   

    // ─── 깔끔한 종료 처리 ──────────────────────────
    std::cout << "\n[Server] 안전하게 종료 중...\n";
    rclcpp::shutdown();       // ROS2 시스템 종료
    if (ros_thread.joinable()) {
        ros_thread.join();    // ROS2 스레드 회수
    }

    return 0;
}
