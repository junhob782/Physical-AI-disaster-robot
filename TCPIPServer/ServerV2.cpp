// =============================================================================
// rescue_bridge_server.cpp
// =============================================================================
// [한 줄 요약] 안드로이드 앱 ↔ ROS2 사이의 TCP/JSON 브릿지 서버
// =============================================================================

// ─── 표준 라이브러리 ──────────────────────────────────
#include <iostream>           // std::cout
#include <string>             // std::string
#include <atomic>             // std::atomic
#include <csignal>            // std::signal, SIGINT, SIGTERM
#include <cstring>            
#include <functional>         // std::function
#include <optional>           // std::optional
#include <unordered_map>      // 해시맵
#include <cerrno>             // errno
#include <thread>             // std::thread

// ─── POSIX 소켓 API ───────────────────────────────────
#include <unistd.h>           // ::close()
#include <arpa/inet.h>        // sockaddr_in, htons, inet_ntop
#include <sys/socket.h>       // 소켓 API 함수들

// ─── 외부 라이브러리 및 ROS2 ──────────────────────────
#include <nlohmann/json.hpp>  // JSON 라이브러지
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using json = nlohmann::json;

// =============================================================================
// 전역 종료 플래그 (시그널 핸들러용)
// =============================================================================
namespace {
    std::atomic<bool> g_running{true};

    void on_signal(int /*sig*/) {
        g_running = false;
    }
}

// =============================================================================
// 클래스 1: Socket — 파일 디스크립터 RAII 래퍼
// =============================================================================
class Socket {
    int fd_ = -1;

public:
    Socket() = default;
    explicit Socket(int fd) : fd_(fd) {}

    // 복사 금지
    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

    // 이동 허용
    Socket(Socket&& o) noexcept : fd_(o.fd_) {
        o.fd_ = -1;
    }
    Socket& operator=(Socket&& o) noexcept {
        if (this != &o) {
            reset();
            fd_ = o.fd_;
            o.fd_ = -1;
        }
        return *this;
    }

    ~Socket() { reset(); }

    int  fd()    const { return fd_; }
    bool valid() const { return fd_ >= 0; }

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
    int      fd_;
    std::string buf_;

public:
    explicit LineReader(int fd) : fd_(fd) {}

    std::optional<std::string> read_line() {
        char chunk[2048];

        while (g_running) {
            if (auto pos = buf_.find('\n'); pos != std::string::npos) {
                std::string line = buf_.substr(0, pos);
                buf_.erase(0, pos + 1);
                return line;
            }

            ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);

            if (n > 0) {
                buf_.append(chunk, n);
                continue;
            }
            if (n == 0) return std::nullopt; // 연결 끊김
            
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue; // 1초 타임아웃으로 인한 대기 재시작
            }
            return std::nullopt;
        }
        return std::nullopt;
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

        timeval tv{1, 0}; // 1초 타임아웃 설정
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
// main — 핸들러 등록 및 실제 ROS2 /cmd_vel 토픽 발행 연동 루프
// =============================================================================
int main(int argc, char* argv[]) {
    // 1. 포트 결정 및 시그널 핸들러 등록
    int port = (argc > 1) ? std::stoi(argv[1]) : 8080;
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    // ─── [핵심 1] ROS2 초기화 및 노드 생성 ──────────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rescue_bridge_node");
    
    // 로봇 모터 노드가 구독하는 "/cmd_vel" 토픽 퍼블리셔 생성!
    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // TCP 서버와 ROS2 가 동시에 돌 수 있도록 별도 스레드로 분리
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });
    // ────────────────────────────────────────────────────────

    MessageRouter router;

    // ─── [핵심 2] 안드로이드 신호를 받으면 실제로 /cmd_vel 발사! ───
    router.on("cmd_vel", [cmd_vel_pub](const json& m) {
        double lx = m.at("linear_x").get<double>();
        double az = m.at("angular_z").get<double>();
        std::cout << "  [cmd_vel 수신 및 발행] 전진: " << lx << " 회전: " << az << "\n";

        // 키보드 제어 노드(teleop)와 완전히 동일한 규칙으로 Twist 데이터 생성
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = lx;     // 전진/후진 속도 (m/s)
        twist_msg.angular.z = az;    // 회전 속도 (rad/s)
        
        // ★ 여기서 하드웨어 드라이버 노드로 진짜 토픽을 쏩니다!
        cmd_vel_pub->publish(twist_msg);

        return json{{"status", "ok"}, {"echo", "cmd_vel"}};
    });

    // 다른 핸들러들은 우선 기본 응답 구조만 유지
    router.on("arm_cmd", [](const json&) { return json{{"status", "ok"}}; });
    router.on("emergency_stop", [](const json&) { return json{{"status", "ok"}}; });
    router.on("nav_goal", [](const json&) { return json{{"status", "ok"}}; });
    router.on("ping", [](const json&) { return json{{"status", "ok"}, {"type", "pong"}}; });

    // ─── 서버 실행 ─────────────────────────────────
    TcpServer server(port);
    if (!server.start()) {
        rclcpp::shutdown();
        return 1; 
    }
    
    std::cout << "[Server] ROS2 브릿지 가동! 안드로이드 조이스틱 명령을 대기합니다.\n";
    server.run(router);   

    // ─── 종료 처리 ─────────────────────────────────
    std::cout << "\n[Server] 안전하게 종료 중...\n";
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return 0;
}
