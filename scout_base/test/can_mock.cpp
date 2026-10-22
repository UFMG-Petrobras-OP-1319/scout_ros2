// Test-only preload: every CAN socket is a local socketpair, never a real bus.
#include <dlfcn.h>
#include <linux/can.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <cstdarg>
#include <cstdlib>
#include <cerrno>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <string>

namespace {
struct State {
  std::mutex mutex;
  std::map<int, int> peers;
  unsigned opens = 0;
  bool send_error_injected = false;
};
State &state() { static auto *value = new State; return *value; }
}
extern "C" int socket(int domain, int type, int protocol) {
  if (domain != PF_CAN) {
    static auto real = reinterpret_cast<int (*)(int, int, int)>(dlsym(RTLD_NEXT, "socket"));
    return real(domain, type, protocol);
  }
  auto &s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  if (++s.opens > 1 && std::getenv("SCOUT_TEST_FAIL_SECOND_CONNECT")) {
    errno = ENETDOWN;
    return -1;
  }
  int pair[2];
  if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, pair)) return -1;
  s.peers[pair[0]] = pair[1];
  return pair[0];
}
extern "C" int ioctl(int fd, unsigned long request, ...) noexcept {
  va_list args;
  va_start(args, request);
  void *argument = va_arg(args, void *);
  va_end(args);
  auto &s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  if (s.peers.count(fd)) {
    auto *ifr = static_cast<ifreq *>(argument);
    if (request == SIOCGIFINDEX) ifr->ifr_ifindex = 1;
    if (request == SIOCGIFHWADDR) ifr->ifr_hwaddr.sa_family = ARPHRD_CAN;
    if (request == SIOCGIFFLAGS) ifr->ifr_flags = IFF_UP;
    return 0;
  }
  return syscall(SYS_ioctl, fd, request, argument);
}
extern "C" int bind(int fd, const sockaddr *address, socklen_t size) {
  auto &s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  auto peer = s.peers.find(fd);
  if (peer != s.peers.end()) {
    const char *signature = std::getenv("SCOUT_TEST_SIGNATURE");
    if (signature && *signature) {
      can_frame frame = {};
      frame.can_id = std::strtoul(signature, nullptr, 0);
      frame.can_dlc = 8;
      frame.data[7] = ((frame.can_id >> 8) + (frame.can_id & 255) + 8) & 255;
      syscall(SYS_write, peer->second, &frame, sizeof(frame));
    }
    const char *drop_file = std::getenv("SCOUT_TEST_DROP_FILE");
    if (s.opens > 1 && drop_file) {
      const std::string path(drop_file);
      const int peer_fd = peer->second;
      std::thread([fd, peer_fd, path]() {
        while (true) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          auto &s = state();
          std::lock_guard<std::mutex> lock(s.mutex);
          auto current = s.peers.find(fd);
          if (current == s.peers.end() || current->second != peer_fd) return;
          if (access(path.c_str(), F_OK) == 0) {
            unlink(path.c_str());
            shutdown(peer_fd, SHUT_RDWR);
            return;
          }
        }
      }).detach();
    }
    return 0;
  }
  return syscall(SYS_bind, fd, address, size);
}
extern "C" ssize_t write(int fd, const void *data, size_t size) {
  auto &s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  const char *error = std::getenv("SCOUT_TEST_SEND_ERRNO");
  if (error && s.opens > 1 && s.peers.count(fd) && !s.send_error_injected) {
    s.send_error_injected = true;
    errno = std::atoi(error);
    return -1;
  }
  return syscall(SYS_write, fd, data, size);
}
extern "C" int close(int fd) {
  auto &s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  auto peer = s.peers.find(fd);
  if (peer != s.peers.end()) {
    syscall(SYS_close, peer->second);
    s.peers.erase(peer);
  }
  return syscall(SYS_close, fd);
}
