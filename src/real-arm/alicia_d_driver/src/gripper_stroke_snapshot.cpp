#include "alicia_d_driver/gripper_stroke_snapshot.hpp"

#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcSocket.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <algorithm>
#include <cerrno>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace alicia_d_driver {
namespace {

struct Endpoint {
    sockaddr_storage address{};
    socklen_t length = 0;

    Endpoint(std::string host, int port) {
        if (port <= 0 || port > 65535)
            throw std::invalid_argument("invalid ROS master port");
        if (host.size() > 2 && host.front() == '[' && host.back() == ']')
            host = host.substr(1, host.size() - 2);
        const int preferred_family = XmlRpc::XmlRpcSocket::s_use_ipv6_ ? AF_INET6 : AF_INET;
        if (host == "localhost")
            host = preferred_family == AF_INET6 ? "::1" : "127.0.0.1";
        auto* ipv4 = reinterpret_cast<sockaddr_in*>(&address);
        auto* ipv6 = reinterpret_cast<sockaddr_in6*>(&address);
        if (inet_pton(AF_INET, host.c_str(), &ipv4->sin_addr) == 1) {
            ipv4->sin_family = AF_INET;
            ipv4->sin_port = htons(port);
            length = sizeof(sockaddr_in);
            return;
        }
        if (inet_pton(AF_INET6, host.c_str(), &ipv6->sin6_addr) == 1) {
            ipv6->sin6_family = AF_INET6;
            ipv6->sin6_port = htons(port);
            length = sizeof(sockaddr_in6);
            return;
        }
        // Resolve a named master once, during node construction and BEFORE
        // any driver timers start. XmlRpcClient::executeNonBlock otherwise
        // hides a blocking getaddrinfo() on every reconnect. The worker uses
        // this sockaddr directly, so cancellation never depends on DNS.
        addrinfo hints{};
        hints.ai_family = preferred_family;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* resolved = nullptr;
        const int error = getaddrinfo(host.c_str(), nullptr, &hints, &resolved);
        if (error != 0)
            throw std::runtime_error(std::string("cannot resolve ROS master: ") + gai_strerror(error));
        std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> owned(resolved, freeaddrinfo);
        if (!resolved || resolved->ai_addrlen > sizeof(address))
            throw std::runtime_error("invalid ROS master address");
        std::memcpy(&address, resolved->ai_addr, resolved->ai_addrlen);
        length = resolved->ai_addrlen;
        if (address.ss_family == AF_INET6)
            ipv6->sin6_port = htons(port);
        else
            ipv4->sin_port = htons(port);
    }
};

class ResolvedClient final : public XmlRpc::XmlRpcClient {
public:
    ResolvedClient(const Endpoint& endpoint, const std::string& host, int port,
                   const std::string& path)
        : XmlRpcClient(host.c_str(), port, path.c_str()), endpoint_(endpoint) {}

private:
    bool doConnect() override {
        const int fd = ::socket(endpoint_.address.ss_family,
                                SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd < 0) return false;
        setfd(fd);
        if (::connect(fd, reinterpret_cast<const sockaddr*>(&endpoint_.address),
                      endpoint_.length) != 0 && errno != EINPROGRESS) {
            close();
            return false;
        }
        return true;
    }
    const Endpoint& endpoint_;
};

double parameter_value(const XmlRpc::XmlRpcValue& response) {
    using Value = XmlRpc::XmlRpcValue;
    if (response.getType() != Value::TypeArray || response.size() != 3 ||
        response[0].getType() != Value::TypeInt || int(response[0]) != 1 ||
        response[1].getType() != Value::TypeString)
        return 0.05;
    // Same scalar types as roscpp getParam(double): int or double, not bool.
    if (response[2].getType() == Value::TypeInt) return int(response[2]);
    if (response[2].getType() == Value::TypeDouble) return double(response[2]);
    return 0.05;
}

}  // namespace

struct GripperStrokeSnapshot::Impl {
    std::atomic<double>& value;
    const std::string host, path, caller, key;
    const int port;
    const Endpoint endpoint;
    const std::chrono::milliseconds period, timeout;
    std::atomic<bool> stopping{false};
    std::mutex wait_mutex, stop_mutex;
    std::condition_variable wake;
    std::thread worker;

    Impl(std::atomic<double>& target, const std::string& host_arg, int port_arg,
         const std::string& path_arg, const std::string& caller_arg,
         const std::string& key_arg, std::chrono::milliseconds period_arg,
         std::chrono::milliseconds timeout_arg)
        : value(target), host(host_arg), path(path_arg.empty() ? "/" : path_arg),
          caller(caller_arg), key(key_arg), port(port_arg), endpoint(host, port),
          period(period_arg), timeout(timeout_arg) {
        if (period.count() <= 0 || timeout.count() <= 0)
            throw std::invalid_argument("parameter refresh periods must be positive");
        refresh();  // Initial bounded value acquisition, before driver timers.
        worker = std::thread(&Impl::run, this);
    }

    ~Impl() { stop(); }

    void stop() {
        std::lock_guard<std::mutex> serial_stop(stop_mutex);
        {
            std::lock_guard<std::mutex> lock(wait_mutex);
            stopping.store(true, std::memory_order_relaxed);
        }
        wake.notify_all();
        if (worker.joinable()) worker.join();
    }

    bool wait(std::chrono::steady_clock::time_point deadline) {
        std::unique_lock<std::mutex> lock(wait_mutex);
        return wake.wait_until(lock, deadline, [this] {
            return stopping.load(std::memory_order_relaxed);
        });
    }

    void refresh() {
        using Clock = std::chrono::steady_clock;
        const auto deadline = Clock::now() + timeout;
        double next = 0.05;
        try {
            ResolvedClient client(endpoint, host, port, path);
            XmlRpc::XmlRpcValue arguments, response;
            arguments[0] = caller;
            arguments[1] = key;
            if (!stopping.load(std::memory_order_relaxed) &&
                client.executeNonBlock("getParam", arguments)) {
                while (!stopping.load(std::memory_order_relaxed) && Clock::now() < deadline) {
                    client._disp.work(0.0);  // One nonblocking socket dispatch.
                    if (client.executeCheckDone(response)) {
                        if (!client.isFault()) next = parameter_value(response);
                        break;
                    }
                    if (wait(std::min(deadline, Clock::now() + std::chrono::milliseconds(10))))
                        break;
                }
            }
            // Only this thread owns/touches its client. No shared ROS client,
            // global parameter mutex, callback queue or cross-thread close.
            client.close();
        } catch (...) {
            // A bad response or transport failure has the same default as
            // the former param<double>() calls; it cannot kill the worker.
        }
        if (!stopping.load(std::memory_order_relaxed))
            value.store(next, std::memory_order_relaxed);
    }

    void run() {
        while (!wait(std::chrono::steady_clock::now() + period)) refresh();
    }
};

GripperStrokeSnapshot::GripperStrokeSnapshot(
    const std::string& host, int port, const std::string& path,
    const std::string& caller, const std::string& key,
    std::chrono::milliseconds period, std::chrono::milliseconds timeout)
    : impl_(new Impl(value_, host, port, path, caller, key, period, timeout)) {}

GripperStrokeSnapshot::~GripperStrokeSnapshot() = default;

void GripperStrokeSnapshot::stop() { impl_->stop(); }

std::string GripperStrokeSnapshot::master_path(const std::string& uri) {
    const auto scheme = uri.find("://");
    const auto slash = uri.find('/', scheme == std::string::npos ? 0 : scheme + 3);
    return slash == std::string::npos ? "/" : uri.substr(slash);
}

}  // namespace alicia_d_driver
