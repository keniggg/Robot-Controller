#include "alicia_d_driver/gripper_stroke_snapshot.hpp"
#include <gtest/gtest.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace {
using Snapshot = alicia_d_driver::GripperStrokeSnapshot;
using Value = XmlRpc::XmlRpcValue;
using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

bool until(const std::function<bool()>& predicate, Ms limit = Ms(2000)) {
    const auto end = Clock::now() + limit;
    do {
        if (predicate()) return true;
        std::this_thread::sleep_for(Ms(5));
    } while (Clock::now() < end);
    return predicate();
}

// A loopback-only fake master. It never connects to ROS or the serial driver.
class Master {
public:
    explicit Master(bool ipv6 = false) {
        fd_ = ::socket(ipv6 ? AF_INET6 : AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
        if (fd_ < 0) throw std::runtime_error("socket failed");
        sockaddr_storage address{};
        socklen_t size;
        if (ipv6) {
            auto* a = reinterpret_cast<sockaddr_in6*>(&address);
            a->sin6_family = AF_INET6;
            a->sin6_addr = in6addr_loopback;
            size = sizeof(*a);
        } else {
            auto* a = reinterpret_cast<sockaddr_in*>(&address);
            a->sin_family = AF_INET;
            a->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            size = sizeof(*a);
        }
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), size) != 0 ||
            ::listen(fd_, 4) != 0) {
            ::close(fd_);
            throw std::runtime_error("loopback bind failed");
        }
        ::getsockname(fd_, reinterpret_cast<sockaddr*>(&address), &size);
        port = ntohs(ipv6 ? reinterpret_cast<sockaddr_in6*>(&address)->sin6_port :
                           reinterpret_cast<sockaddr_in*>(&address)->sin_port);
        reply(Value(0.08));
        worker_ = std::thread([this] { run(); });
    }
    ~Master() {
        stopped_.store(true);
        worker_.join();
        ::close(fd_);
    }
    void reply(Value payload, int code = 1, Value message = Value("test")) {
        Value response;
        response[0] = code;
        response[1] = message;
        response[2] = payload;
        std::lock_guard<std::mutex> lock(mutex_);
        body_ = "<?xml version=\"1.0\"?><methodResponse><params><param>" +
            response.toXml() + "</param></params></methodResponse>";
    }
    std::string request() {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_request_;
    }
    int port;
    std::atomic<int> requests{0};
    std::atomic<bool> stall{false};
private:
    void run() {
        while (!stopped_.load()) {
            pollfd listen_poll{fd_, POLLIN, 0};
            if (::poll(&listen_poll, 1, 10) <= 0) continue;
            const int client = ::accept4(fd_, nullptr, nullptr, SOCK_NONBLOCK);
            if (client < 0) continue;
            std::string request;
            const auto deadline = Clock::now() + Ms(2000);
            while (!stopped_.load() && Clock::now() < deadline &&
                   request.find("</methodCall>") == std::string::npos) {
                pollfd read_poll{client, POLLIN, 0};
                if (::poll(&read_poll, 1, 10) <= 0) continue;
                char buffer[4096];
                const auto n = ::recv(client, buffer, sizeof(buffer), 0);
                if (n <= 0) break;
                request.append(buffer, n);
            }
            std::string body;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_request_ = request;
                body = body_;
            }
            requests.fetch_add(1);
            while (stall.load() && !stopped_.load()) std::this_thread::sleep_for(Ms(5));
            if (!stopped_.load()) {
                const std::string response = "HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nContent-Length: " +
                    std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
                ::send(client, response.data(), response.size(), MSG_NOSIGNAL);
            }
            ::close(client);
        }
    }
    int fd_;
    std::atomic<bool> stopped_{false};
    std::mutex mutex_;
    std::string body_, last_request_;
    std::thread worker_;
};

TEST(GripperStrokeSnapshot, DynamicDeletionTypesAndNumericSemantics) {
    Master master;
    Snapshot snapshot("127.0.0.1", master.port, "/rosmaster", "/test_driver",
                      "/test_driver/gripper_stroke_m", Ms(20), Ms(250));
    ASSERT_DOUBLE_EQ(0.08, snapshot.load());
    const auto request = master.request();
    EXPECT_NE(std::string::npos, request.find("POST /rosmaster HTTP/1.1"));
    EXPECT_NE(std::string::npos, request.find("<methodName>getParam</methodName>"));
    EXPECT_NE(std::string::npos, request.find("/test_driver/gripper_stroke_m"));
    master.reply(Value(0.09));
    ASSERT_TRUE(until([&] { return snapshot.load() == 0.09; }));
    master.reply(Value(0), -1); // absent/deleted key
    ASSERT_TRUE(until([&] { return snapshot.load() == 0.05; }));
    master.reply(Value(0.07)); // recreation
    ASSERT_TRUE(until([&] { return snapshot.load() == 0.07; }));
    for (const auto& payload : {Value("bad"), Value(true)}) {
        master.reply(payload);
        ASSERT_TRUE(until([&] { return snapshot.load() == 0.05; }));
        master.reply(Value(0.07));
        ASSERT_TRUE(until([&] { return snapshot.load() == 0.07; }));
    }
    master.reply(Value(0.12), 0); // ROS RPC error
    ASSERT_TRUE(until([&] { return snapshot.load() == 0.05; }));
    for (const auto& payload : {Value(2), Value(0.0), Value(-0.03)}) {
        const double expected = payload.getType() == Value::TypeInt ? int(payload) : double(payload);
        master.reply(payload);
        ASSERT_TRUE(until([&] { return snapshot.load() == expected; }));
    }
    master.reply(Value(0.12), 1, Value(42)); // malformed ROS envelope
    ASSERT_TRUE(until([&] { return snapshot.load() == 0.05; }));
}

TEST(GripperStrokeSnapshot, StalledMasterTimesOutAndStopCancelsInflightRead) {
    Master master;
    master.stall.store(true);
    const auto initial = Clock::now();
    {
        Snapshot snapshot("localhost", master.port, "/", "/test", "/stroke", Ms(20), Ms(80));
        EXPECT_DOUBLE_EQ(0.05, snapshot.load());
        EXPECT_LT(std::chrono::duration<double>(Clock::now() - initial).count(), 1.0);
        snapshot.stop();
        snapshot.stop();
    }
    master.stall.store(false);
    ASSERT_TRUE(until([&] { return master.requests.load() >= 1; }));
    Snapshot snapshot("127.0.0.1", master.port, "/", "/test", "/stroke", Ms(40), Ms(5000));
    ASSERT_DOUBLE_EQ(0.08, snapshot.load());
    const int before = master.requests.load();
    master.stall.store(true);
    ASSERT_TRUE(until([&] { return master.requests.load() > before; }));
    const auto start = Clock::now();
    snapshot.stop();
    EXPECT_LT(std::chrono::duration<double>(Clock::now() - start).count(), 0.5);
}

TEST(GripperStrokeSnapshot, RefusedConnectionAndDestructorAreBounded) {
    int closed_port;
    { Master master; closed_port = master.port; }
    const auto start = Clock::now();
    {
        Snapshot snapshot("127.0.0.1", closed_port, "/", "/test", "/stroke", Ms(20), Ms(100));
        EXPECT_DOUBLE_EQ(0.05, snapshot.load());
    }
    EXPECT_LT(std::chrono::duration<double>(Clock::now() - start).count(), 1.0);
    Master master;
    std::unique_ptr<Snapshot> snapshot(new Snapshot("127.0.0.1", master.port, "/",
        "/test", "/stroke", Ms(40), Ms(5000)));
    const int before = master.requests.load();
    master.stall.store(true);
    ASSERT_TRUE(until([&] { return master.requests.load() > before; }));
    const auto destroy = Clock::now();
    snapshot.reset();
    EXPECT_LT(std::chrono::duration<double>(Clock::now() - destroy).count(), 0.5);
}

TEST(GripperStrokeSnapshot, Ipv6AndMasterUriPaths) {
    EXPECT_EQ("/", Snapshot::master_path("http://localhost:11311"));
    EXPECT_EQ("/rosmaster/", Snapshot::master_path("http://[::1]:11311/rosmaster/"));
    Master master(true);
    Snapshot snapshot("[::1]", master.port, "/rosmaster/", "/test", "/stroke");
    EXPECT_DOUBLE_EQ(0.08, snapshot.load());
}
}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
