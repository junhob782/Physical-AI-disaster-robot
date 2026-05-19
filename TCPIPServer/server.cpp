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
//   빌드: g++ -std=c++17 -O2 rescue_bridge_server.cpp -o bridge -pthread
//   실행: ./bridge          (기본 포트 8080)
//        ./bridge 9000      (포트 지정)
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
// [지원 메시지 타입] (자세한 형식은 각 핸들러 주석 참고)
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
// [왜 필요한가]
//   Ctrl+C 가 입력되면 OS가 프로세스에 SIGINT 시그널을 보낸다.
//   시그널 핸들러에서 종료 처리를 직접 하면 위험하므로(시그널 컨텍스트는
//   대부분의 함수 호출이 금지됨), 단순히 플래그만 false 로 바꾸고
//   메인 루프들이 주기적으로 이 플래그를 확인해 스스로 종료한다.
//
// [왜 std::atomic 인가]
//   시그널 핸들러는 별도 컨텍스트에서 실행 → 일반 변수 접근은 데이터 레이스 발생.
//   atomic<bool> 은 시그널-안전(signal-safe) 보장됨.
//
// [왜 익명 namespace 인가]
//   이 파일 내부에서만 쓰이는 전역임을 명시. (= C 언어의 static 키워드 역할)
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
// [왜 필요한가]
//   POSIX 소켓은 정수형 fd(파일 디스크립터)다. 사용 끝나면 명시적으로 ::close()
//   해야 OS 자원 누수가 안 생긴다. C 스타일로 직접 관리하면 예외나 중간 return 시
//   close() 누락되기 쉽다.
//
// [어떻게 해결하는가 — RAII]
//   소멸자(~Socket) 가 자동으로 ::close() 호출 → 객체가 사라지면 fd 무조건 닫힘.
//   "객체 수명 = 자원 수명" 패턴을 RAII (Resource Acquisition Is Initialization) 라 함.
//   std::unique_ptr 와 같은 개념. 자바·파이썬의 try-with-resources / context manager 와 유사.
//
// [복사는 금지, 이동만 허용]
//   같은 fd 를 두 객체가 소유하면 close() 가 두 번 호출되어 충돌·예측 불가 동작.
//   → 복사 생성자/대입 = delete, 이동 생성자/대입은 허용.
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
// [⭐ 가장 중요한 학습 포인트 — TCP 스트림 프레이밍]
//
// TCP 는 "바이트 스트림" 프로토콜이다. 메시지 경계 개념이 없다.
//
//   예시 1) 클라이언트가 빠르게 두 번 보내면 서버 recv() 한 번에 합쳐서 받을 수 있음:
//       클라이언트: send("{\"a\":1}\n")
//                   send("{\"b\":2}\n")
//       서버 recv: "{\"a\":1}\n{\"b\":2}\n"   ← 합쳐서 한 번에
//
//   예시 2) 반대로 한 메시지가 둘로 쪼개져 도착 가능:
//       1차 recv: "{\"msg_type\":\"cmd_v"
//       2차 recv: "el\",\"linear_x\":0.5}\n"
//
// 그래서 "recv() 한 번 = 메시지 한 개" 가정은 깨진다.
//
// [해결책 — 구분자 기반 프레이밍]
//   메시지 끝마다 '\n' 을 약속 (JSON Lines 프로토콜).
//   받는 쪽은:
//     1) recv() 결과를 내부 버퍼에 계속 누적
//     2) 버퍼 안에 '\n' 있는지 검색
//     3) 있으면 '\n' 앞까지 잘라 한 줄 반환, 처리한 부분은 버퍼에서 제거
//     4) 없으면 더 recv() 받아 누적
//
// [반환 타입]
//   std::optional<std::string>:
//     - 정상 줄: 문자열 반환
//     - 연결 끊김 / 오류: std::nullopt 반환 (값 없음 의미)
class LineReader {
    int         fd_;     // 읽을 소켓의 fd
    std::string buf_;    // 누적 버퍼 (다음 메시지 일부가 미리 들어와 있을 수 있음)

public:
    explicit LineReader(int fd) : fd_(fd) {}

    std::optional<std::string> read_line() {
        char chunk[2048];   // recv() 한 번에 받을 임시 버퍼

        while (g_running) {
            // [1단계] 누적 버퍼에 이미 '\n' 이 있는지 검사
            //          (지난 번 recv 에서 여러 메시지가 한꺼번에 왔을 수 있음)
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
                // while 조건으로 돌아감
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
// [왜 필요한가]
//   원본 코드처럼 if/else if 사슬로 처리하면 새 명령 추가 시 main() 안을 매번
//   수정해야 한다. 이는 개방-폐쇄 원칙(OCP) 위반.
//
// [어떻게 해결하는가 — 핸들러 등록 패턴]
//     router.on("cmd_vel",        [](const json& m){ ... });
//     router.on("arm_cmd",        [](const json& m){ ... });
//     router.on("emergency_stop", [](const json& m){ ... });
//   새 명령 추가 시 핸들러만 등록하면 라우터가 알아서 호출.
//
// [핸들러 시그니처]
//   입력: const json&  — 수신된 JSON 메시지 전체
//   출력: json         — 클라이언트에 돌려줄 응답
//
// [예외 처리 일원화]
//   JSON 파싱·타입·누락 필드 오류를 라우터가 모두 잡아서 표준 에러 응답으로 변환.
//   덕분에 개별 핸들러는 정상 케이스(happy path) 만 신경 쓰면 됨.
class MessageRouter {
public:
    // 핸들러 타입 별칭. std::function 은 람다·함수 포인터를 담는 컨테이너.
    using Handler = std::function<json(const json&)>;

    // 핸들러 등록. type 문자열에 대응되는 람다를 맵에 저장.
    // (std::move 로 불필요한 복사 방지)
    void on(std::string type, Handler h) {
        handlers_[std::move(type)] = std::move(h);
    }

    // 수신된 JSON 문자열을 처리하고 응답 JSON 을 반환.
    json dispatch(const std::string& raw) const {
        try {
            // 문자열 → JSON 객체 파싱
            json m = json::parse(raw);

            // msg_type 필드 추출. 없으면 빈 문자열로 처리.
            // ( m.value("키", 기본값) = contains 체크 + 추출을 한 줄로 )
            std::string type = m.value("msg_type", "");

            // 해당 타입의 핸들러 찾기
            auto it = handlers_.find(type);
            if (it == handlers_.end()) {
                return err("unknown msg_type: '" + type + "'");
            }
            // 핸들러 실행 후 그 결과(json) 반환
            return it->second(m);
        }
        // 아래는 nlohmann/json 이 던지는 예외 3종:
        catch (const json::parse_error& e) {
            // JSON 문법 자체가 깨진 경우 (괄호 누락 등)
            return err(std::string("invalid JSON: ") + e.what());
        } catch (const json::type_error& e) {
            // 필드 타입이 기대와 다른 경우 (예: 숫자 자리에 문자열)
            return err(std::string("type error: ") + e.what());
        } catch (const json::out_of_range& e) {
            // 핸들러에서 .at("키") 했는데 키가 없는 경우
            return err(std::string("missing field: ") + e.what());
        }
    }

private:
    // 표준 에러 응답 형식
    static json err(std::string msg) {
        return {{"status", "error"}, {"message", std::move(msg)}};
    }

    // type 문자열 → 핸들러 람다 매핑
    std::unordered_map<std::string, Handler> handlers_;
};


// =============================================================================
// 클래스 4: TcpServer — 소켓 라이프사이클 관리
// =============================================================================
// [TCP 서버 표준 4단계]
//   1. socket()  - 소켓 fd 생성 (TCP 는 SOCK_STREAM 타입)
//   2. bind()    - 특정 IP : 포트에 묶기
//   3. listen()  - 연결 대기(passive) 모드로 전환
//   4. accept()  - 클라이언트 연결 받기 (블로킹 함수)
//
// [재연결 지원]
//   accept() 가 무한 루프 안에 있어 클라이언트가 끊겨도 다음 연결을 다시 받음.
//   (원본 코드는 accept() 가 루프 밖에 있어 한 번 끊기면 서버 종료되는 버그였음)
//
// [SO_RCVTIMEO]
//   accept()/recv() 는 기본 블로킹 → Ctrl+C 시그널이 와도 못 깨움.
//   1초 timeout 을 걸어 주기적으로 깨어나 g_running 플래그 재확인.
class TcpServer {
public:
    explicit TcpServer(int port) : port_(port) {}

    // 소켓 생성·옵션 설정·bind·listen 1회 수행.
    // 성공 true, 실패 false (에러 메시지는 perror 로).
    bool start() {
        // [1단계] 소켓 생성
        //   AF_INET     = IPv4
        //   SOCK_STREAM = TCP (vs SOCK_DGRAM = UDP)
        //   0           = 프로토콜 자동 선택 (TCP)
        int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            std::perror("socket");
            return false;
        }
        listen_ = Socket(fd);  // RAII 래퍼에 위탁 → 이후 자동 close 보장

        // [옵션 1] SO_REUSEADDR
        //   서버 종료 직후 같은 포트 재바인딩 허용.
        //   없으면 OS 가 1~2분간 TIME_WAIT 상태로 잡고 있어 재실행 시
        //   "Address already in use" 발생.
        int opt = 1;
        ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // [옵션 2] SO_RCVTIMEO
        //   recv()/accept() 가 1초마다 깨어나도록 timeout 설정.
        //   종료 플래그(g_running) 를 주기적으로 확인할 수 있게 함.
        timeval tv{1, 0};   // 1초 0마이크로초
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // [2단계] bind — 어떤 IP 의 어떤 포트에서 들을지 지정
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;       // 모든 네트워크 인터페이스
        addr.sin_port        = htons(port_);     // 호스트 → 네트워크 바이트 순서 변환

        if (::bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
            std::perror("bind");
            return false;
        }

        // [3단계] listen — 대기 모드 전환
        //   두 번째 인자(1) = 백로그 큐 크기. 동시에 대기 가능한 미수락 연결 수.
        //   클라이언트 한 대만 받으므로 1 로 충분.
        if (::listen(fd, 1) < 0) {
            std::perror("listen");
            return false;
        }

        std::cout << "[Server] port " << port_ << " LISTEN\n";
        return true;
    }

    // 메인 서버 루프 — 종료 플래그가 켜질 때까지 클라이언트를 계속 받음.
    void run(const MessageRouter& router) {
        while (g_running) {
            // [4단계] accept — 클라이언트 연결 대기 + 수락
            sockaddr_in caddr{};                  // 연결된 클라이언트의 주소가 채워질 곳
            socklen_t   clen = sizeof(caddr);
            int cfd = ::accept(listen_.fd(), (sockaddr*)&caddr, &clen);

            if (cfd < 0) {
                // 1초 timeout 이라면 다시 루프 (종료 플래그 확인 목적)
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                if (!g_running) break;
                std::perror("accept");
                continue;
            }

            // 클라이언트 소켓에도 동일하게 1초 timeout 적용
            // (LineReader 가 주기적으로 g_running 체크할 수 있게)
            timeval tv{1, 0};
            ::setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            // 연결 정보 출력 (디버깅용)
            char ip[INET_ADDRSTRLEN];
            ::inet_ntop(AF_INET, &caddr.sin_addr, ip, sizeof(ip));
            std::cout << "[Server] client connected: " << ip << "\n";

            // 이 클라이언트 처리에 전념. 끊길 때까지 블로킹.
            serve_client(Socket(cfd), router);

            std::cout << "[Server] client disconnected, waiting for next...\n";
            // 다시 루프 시작 → accept() 로 다음 클라이언트 대기
        }
    }

private:
    // 하나의 클라이언트 연결을 처리하는 내부 함수.
    // sock 을 값으로 받음 → 함수 종료 시 RAII 로 자동 close.
    void serve_client(Socket sock, const MessageRouter& router) {
        LineReader reader(sock.fd());

        while (g_running) {
            // 한 줄(메시지) 받기
            auto line = reader.read_line();
            if (!line) break;            // 연결 끊김 또는 오류
            if (line->empty()) continue; // 빈 줄은 무시

            // 라우터에 위임 → 응답 받기
            json resp = router.dispatch(*line);

            // 응답을 줄 단위로 송신 (반드시 '\n' 종결)
            std::string out = resp.dump() + "\n";

            // MSG_NOSIGNAL: 클라이언트가 갑자기 끊겼을 때 SIGPIPE 안 받게.
            // (대신 send 의 반환값으로 판단)
            ssize_t sent = ::send(sock.fd(), out.data(), out.size(), MSG_NOSIGNAL);
            if (sent <= 0) break;        // 송신 실패 = 연결 끊김
        }
    }

    int    port_;       // 바인딩할 포트 번호
    Socket listen_;     // 리스닝 소켓 (RAII)
};


// =============================================================================
// main — 핸들러 등록 + 서버 실행
// =============================================================================
// [흐름]
//   1. 명령줄 인자로 포트 받기 (없으면 8080)
//   2. 시그널 핸들러 등록 (Ctrl+C 대응)
//   3. MessageRouter 에 핸들러 5종 등록
//   4. TcpServer 시작 → 무한 루프
//
// [현재 한계 — 다음 단계 작업]
//   각 핸들러는 cout 만 함. 실제 ROS2 토픽 발행은 TODO 위치에 구현 필요.
//   rclcpp::Node 상속한 BridgeNode 클래스를 만들고, publisher 들을 멤버로 두어야 함.
//   colcon 빌드 구성(CMakeLists, package.xml) 과 함께 별도 작업.
int main(int argc, char* argv[]) {
    // 1. 포트 결정 및 시그널 핸들러 등록
    int port = (argc > 1) ? std::stoi(argv[1]) : 8080;
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    // ─── [추가됨] ROS2 초기화 및 노드 생성 ──────────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rescue_bridge_node");
    
    // 로봇의 바퀴로 명령을 보낼 Publisher 생성 (/cmd_vel 토픽)
    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // TCP 서버(while 루프)가 블로킹되더라도 ROS2가 뒤에서 돌아갈 수 있도록 별도 스레드로 분리
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });
    // ────────────────────────────────────────────────────────

    MessageRouter router;

    // ─────────────────────────────────────────────
    // [cmd_vel] 로봇 주행 명령 수신 및 ROS2 퍼블리시
    // ─────────────────────────────────────────────
    // 람다 함수에 cmd_vel_pub 캡처를 추가하여 내부에서 ROS2 명령을 쏠 수 있게 합니다.
    router.on("cmd_vel", [cmd_vel_pub](const json& m) {
        double lx = m.at("linear_x").get<double>();
        double az = m.at("angular_z").get<double>();
        std::cout << "  [cmd_vel 수신] 전진: " << lx << " 회전: " << az << "\n";

        // [핵심] JSON 속도 값을 ROS2 Twist 메시지로 변환하여 로봇으로 전송!
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = lx;
        twist_msg.angular.z = az;
        cmd_vel_pub->publish(twist_msg);

        return json{{"status", "ok"}, {"echo", "cmd_vel"}};
    });

    // 다른 핸들러들은 우선 기존처럼 터미널 출력만 유지
    router.on("arm_cmd", [](const json& m) {
        /* ... 기존 로직 ... */
        return json{{"status", "ok"}, {"echo", "arm_cmd"}};
    });
    router.on("emergency_stop", [](const json&) {
        /* ... 기존 로직 ... */
        return json{{"status", "ok"}, {"echo", "emergency_stop"}};
    });
    router.on("nav_goal", [](const json& m) {
        /* ... 기존 로직 ... */
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

    // ─── [추가됨] 깔끔한 종료 처리 ─────────────────
    std::cout << "\n[Server] 안전하게 종료 중...\n";
    rclcpp::shutdown();       // ROS2 시스템 종료
    if (ros_thread.joinable()) {
        ros_thread.join();    // ROS2 스레드 회수
    }

    return 0;
}
