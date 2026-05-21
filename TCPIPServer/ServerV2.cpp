// =============================================================================
// rescue_bridge_server.cpp
// =============================================================================

#include <iostream>
#include <string>
#include <atomic>
#include <csignal>
#include <cstring>
#include <functional>
#include <optional>
#include <unordered_map>
#include <cerrno>
#include <thread>
#include <mutex>
#include <algorithm>
#include <vector>

// POSIX
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

// JSON
#include <nlohmann/json.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

// ★ 실제 서보 메시지 타입
#include <ros_robot_controller_msgs/msg/servos_position.hpp>
#include <ros_robot_controller_msgs/msg/servo_position.hpp>

using json = nlohmann::json;

// =============================================================================
// 전역 상태
// =============================================================================

namespace {
    std::atomic<bool> g_running{true};
    std::atomic<int> g_client_fd{-1};
    std::mutex g_send_mutex;

    void on_signal(int) {
        g_running = false;
    }
}

// =============================================================================
// Socket
// =============================================================================

class Socket {
    int fd_ = -1;

public:
    Socket() = default;

    explicit Socket(int fd) : fd_(fd) {}

    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

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

    ~Socket() {
        reset();
    }

    int fd() const {
        return fd_;
    }

    bool valid() const {
        return fd_ >= 0;
    }

    void reset() {
        if (fd_ >= 0) {
            ::close(fd_);

            if (g_client_fd == fd_) {
                g_client_fd = -1;
            }

            fd_ = -1;
        }
    }
};

// =============================================================================
// LineReader
// =============================================================================

class LineReader {
    int fd_;
    std::string buf_;

public:
    explicit LineReader(int fd) : fd_(fd) {}

    std::optional<std::string> read_line() {

        char chunk[2048];

        while (g_running) {

            if (auto pos = buf_.find('\n'); pos != std::string::npos) {

                std::string line = buf_.substr(0, pos);

                buf_.erase(0, pos + 1);

                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                return line;
            }

            std::memset(chunk, 0, sizeof(chunk));

            ssize_t n = ::recv(fd_, chunk, sizeof(chunk) - 1, 0);

            if (n > 0) {
                buf_.append(chunk, n);
                continue;
            }

            if (n == 0) {
                return std::nullopt;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            return std::nullopt;
        }

        return std::nullopt;
    }
};

// =============================================================================
// MessageRouter
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
                return err("unknown msg_type");
            }

            return it->second(m);

        } catch (...) {

            return err("json parse error");
        }
    }

private:

    static json err(std::string msg) {
        return {
            {"status", "error"},
            {"message", std::move(msg)}
        };
    }

    std::unordered_map<std::string, Handler> handlers_;
};

// =============================================================================
// TcpServer
// =============================================================================

class TcpServer {

public:

    explicit TcpServer(int port)
        : port_(port) {}

    bool start() {

        int fd = ::socket(AF_INET, SOCK_STREAM, 0);

        if (fd < 0) {
            std::perror("socket");
            return false;
        }

        listen_ = Socket(fd);

        int opt = 1;

        ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        timeval tv{1, 0};

        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        sockaddr_in addr{};

        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);

        if (::bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
            std::perror("bind");
            return false;
        }

        if (::listen(fd, 1) < 0) {
            std::perror("listen");
            return false;
        }

        std::cout << "[Server] LISTEN OK\n";

        return true;
    }

    void run(const MessageRouter& router) {

        while (g_running) {

            sockaddr_in caddr{};
            socklen_t clen = sizeof(caddr);

            int cfd = ::accept(listen_.fd(), (sockaddr*)&caddr, &clen);

            if (cfd < 0) {

                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }

                continue;
            }

            std::cout << "[Server] Client Connected\n";

            g_client_fd = cfd;

            serve_client(Socket(cfd), router);

            std::cout << "[Server] Client Disconnected\n";
        }
    }

private:

    void serve_client(Socket sock, const MessageRouter& router) {

        LineReader reader(sock.fd());

        while (g_running) {

            auto line = reader.read_line();

            if (!line) break;

            if (line->empty()) continue;

            std::cout << "📥 " << *line << std::endl;

            json resp = router.dispatch(*line);

            std::string out = resp.dump() + "\n";

            std::lock_guard<std::mutex> lock(g_send_mutex);

            ssize_t sent = ::send(
                sock.fd(),
                out.data(),
                out.size(),
                MSG_NOSIGNAL
            );

            if (sent <= 0) break;
        }
    }

    int port_;
    Socket listen_;
};

// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char* argv[]) {

    int port = (argc > 1) ? std::stoi(argv[1]) : 8080;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    // ROS2 INIT
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>("rescue_bridge_node");

    // 주행 publisher
    auto cmd_vel_pub =
        node->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

    // ★ 실제 서보 publisher
    auto arm_joint_pub =
        node->create_publisher<
            ros_robot_controller_msgs::msg::ServosPosition>(
                "/ros_robot_controller/bus_servo/set_position",
                10);

    // 액션 publisher
    auto arm_action_pub =
        node->create_publisher<std_msgs::msg::String>(
            "/arm_action", 10);

    // 비상정지 publisher
    auto emergency_pub =
        node->create_publisher<std_msgs::msg::Bool>(
            "/emergency_stop", 10);

    // ROS spin thread
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });

    MessageRouter router;

    // =========================================================================
    // cmd_vel
    // =========================================================================

    router.on("cmd_vel", [cmd_vel_pub](const json& m) {

        double lx = m.at("linear_x").get<double>();
        double az = m.at("angular_z").get<double>();

        geometry_msgs::msg::Twist twist;

        twist.linear.x = lx;
        twist.angular.z = az;

        cmd_vel_pub->publish(twist);

        return json{{"status", "ok"}};
    });

    // =========================================================================
    // arm_cmd
    // =========================================================================

    router.on("arm_cmd", [arm_joint_pub](const json& m) {

        ros_robot_controller_msgs::msg::ServosPosition servo_msg;

        servo_msg.duration = 0.3;

        std::vector<uint16_t> joints = {

            static_cast<uint16_t>(m.at("joint1").get<double>()),
            static_cast<uint16_t>(m.at("joint2").get<double>()),
            static_cast<uint16_t>(m.at("joint3").get<double>()),
            static_cast<uint16_t>(m.at("joint4").get<double>()),
            static_cast<uint16_t>(m.at("joint5").get<double>()),
            static_cast<uint16_t>(m.at("joint6").get<double>())
        };

        for (size_t i = 0; i < joints.size(); i++) {

            ros_robot_controller_msgs::msg::ServoPosition servo;

            servo.id = i + 1;
            servo.position = joints[i];

            servo_msg.position.push_back(servo);
        }

        arm_joint_pub->publish(servo_msg);

        std::cout << "✅ Servo Publish Success" << std::endl;

        return json{{"status", "ok"}};
    });

    // =========================================================================
    // arm_action
    // =========================================================================

    router.on("arm_action", [arm_action_pub](const json& m) {

        std::string action_name =
            m.at("action_name").get<std::string>();

        std_msgs::msg::String msg;

        msg.data = action_name;

        arm_action_pub->publish(msg);

        return json{{"status", "ok"}};
    });

    // =========================================================================
    // emergency_stop
    // =========================================================================

    router.on("emergency_stop",
        [cmd_vel_pub, emergency_pub](const json&) {

        geometry_msgs::msg::Twist stop_msg;

        cmd_vel_pub->publish(stop_msg);

        std_msgs::msg::Bool em;

        em.data = true;

        emergency_pub->publish(em);

        return json{{"status", "ok"}};
    });

    // =========================================================================
    // ping
    // =========================================================================

    router.on("ping", [](const json&) {

        return json{
            {"status", "ok"},
            {"type", "pong"}
        };
    });

    // =========================================================================
    // SERVER START
    // =========================================================================

    TcpServer server(port);

    if (!server.start()) {

        rclcpp::shutdown();

        return 1;
    }

    std::cout << "[Server] START\n";

    server.run(router);

    // 종료 처리
    g_running = false;

    rclcpp::shutdown();

    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return 0;
}
