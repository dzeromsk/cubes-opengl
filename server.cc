// Copyright (c) 2016 Dominik Zeromski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gflags/gflags.h>
#include <glm/glm.hpp>
#include <glog/logging.h>
#include <uv.h>

#include <functional>
#include <set>

#pragma pack(push, 1)
struct State {
  glm::vec3 position;
  glm::vec4 orientation;
  uint32_t interacting;
};
#pragma pack(pop)

#define container_of(ptr, type, member)                                        \
  ((type *)((char *)(ptr)-offsetof(type, member)))

typedef std::vector<State> Frame;
typedef std::vector<Frame> Frames;

DEFINE_int32(cubes_count, 901, "Cubes count per frame");
DEFINE_string(logfile, "models.log", "Path to log file");

static Frames &Log() {
  static Frames log;
  return log;
}

static void ReadLog(std::string filename) {
  FILE *f = nullptr;
  CHECK(f = fopen(filename.c_str(), "rb"));
  Log().clear();
  for (int frame = 1; frame; frame++) {
    std::vector<State> cubes;
    for (int i = 0; i < FLAGS_cubes_count; i++) {
      State s;
      if (fread(&s, sizeof(State), 1, f) != 1) {
        break;
      }
      cubes.push_back(s);
    }
    if (feof(f)) {
      break;
    }
    Log().push_back(cubes);
  }
  fclose(f);
}

class Loop {
  friend class UDP;
  friend class Timer;

public:
  Loop() { CHECK(uv_loop_init(&loop_) == 0); }
  ~Loop() { uv_loop_close(&loop_); }

  int Run() { return uv_run(&loop_, UV_RUN_DEFAULT); }
  void Stop() { uv_stop(&loop_); }

private:
  uv_loop_t loop_;
};

typedef struct {
  uv_udp_send_t req;
  char *data;
} udp_send_ctx_t;

class UDP {
public:
  UDP(Loop &loop) {
    CHECK(uv_udp_init(&loop.loop_, &socket_) == 0);
    socket_.data = this;
  }

  ~UDP() { uv_udp_recv_stop(&socket_); }

  void Listen(const char *ip, int port) {
    struct sockaddr_in addr;
    CHECK(uv_ip4_addr(ip, port, &addr) == 0);
    CHECK(uv_udp_bind(&socket_, (const struct sockaddr *)&addr, 0) == 0);
    CHECK(uv_udp_recv_start(&socket_, UDP::Alloc, UDP::ReceiveWrapper) == 0);
  }

  typedef std::function<void()> CloseFunc;
  typedef std::function<void(uv_buf_t, const struct sockaddr *, unsigned)>
      ReceiveFunc;

  void OnReceive(ReceiveFunc on_receive) { receive_ = std::move(on_receive); }

  int Send(const uv_buf_t bufs[], unsigned int nbufs,
           const struct sockaddr *addr) {
    udp_send_ctx_t *req = (udp_send_ctx_t *)malloc(sizeof(*req));
    uv_udp_send(&(req->req), &socket_, bufs, nbufs, addr, UDP::Send);
  }

private:
  static void CloseWrapper(uv_handle_t *handle) {
    ((UDP *)handle->data)->close_();
  }

  static void ReceiveWrapper(uv_udp_t *handle, ssize_t nread,
                             const uv_buf_t *buf, const struct sockaddr *addr,
                             unsigned flags) {
    if (nread > 0 && addr != nullptr) {
      uv_buf_t data = {buf->base, size_t(nread)};
      ((UDP *)handle->data)->receive_(data, addr, flags);
    }
  }

  static void Alloc(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf) {
    static char slab[65536];
    *buf = uv_buf_init(slab, sizeof(slab));
  }

  static void Send(uv_udp_send_t *req, int status) {
    udp_send_ctx_t *ctx = container_of(req, udp_send_ctx_t, req);
    free(ctx);
  }

  CloseFunc close_;
  ReceiveFunc receive_;

  uv_udp_t socket_;
};

struct Addr {
  uint32_t ip;
  uint16_t port;

  Addr(const struct sockaddr *addr) {
    ip = ((struct sockaddr_in *)addr)->sin_addr.s_addr;
    port = ntohs(((struct sockaddr_in *)addr)->sin_port);
  }

  bool operator<(const Addr &rhs) const {
    return ip < rhs.ip || (!(rhs.ip < ip) && port < rhs.port);
  }

  bool operator!=(const Addr &rhs) const {
    return ip != rhs.ip || port != rhs.port;
  }

  void Sock(struct sockaddr_in *addr) const {
    memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    addr->sin_addr.s_addr = ip;
  }
};

class UDPServer {
public:
  UDPServer() : socket_(loop_) {}
  typedef std::function<void(const uv_buf_t, const Addr)> ReceiveFunc;
  UDPServer(ReceiveFunc on_receive)
      : socket_(loop_), on_receive_(std::move(on_receive)) {
    socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                          unsigned flags) { on_receive_(buf, addr); });
  }

  int ListenAndServe(const char *ip, int port) {
    socket_.Listen(ip, port);
    return loop_.Run();
  }

  int Send(const Addr &client, const uv_buf_t *buf) {
    struct sockaddr_in addr;
    client.Sock(&addr);
    socket_.Send(buf, 1, (struct sockaddr *)&addr);
  }

protected:
  Loop loop_;
  UDP socket_;
  ReceiveFunc on_receive_;
};

class Timer {
public:
  Timer(Loop &loop) {
    timer_.data = this;
    CHECK(uv_timer_init(&loop.loop_, &timer_) == 0);
  }

  Timer(Loop *loop, std::function<void(void)> callback, uint64_t repeat)
      : callback_(std::move(callback)) {
    timer_.data = this;
    CHECK(uv_timer_init(&loop->loop_, &timer_) == 0);
    CHECK(uv_timer_start(&timer_, Timer::Wrapper, 1, repeat) == 0);
  }

  ~Timer() { uv_timer_stop(&timer_); }

  void Start(std::function<void(void)> callback, uint64_t repeat) {
    callback_ = std::move(callback);
    CHECK(uv_timer_start(&timer_, Timer::Wrapper, 1, repeat) == 0);
  }

private:
  static void Wrapper(uv_timer_t *handle) {
    ((Timer *)handle->data)->callback_();
  }
  std::function<void(void)> callback_;

  uv_timer_t timer_;
};

class GameServer : public UDPServer {
public:
  GameServer() : timer_(loop_) {
    socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                          unsigned flags) { OnReceive(buf, addr); });

    timer_.Start([&] { OnTick(); }, 16);
  }

private:
  void OnTick() {
    Frame &frame = Next();
    uv_buf_t response = {(char *)frame.data(), frame.size() * sizeof(State)};
    for (const auto &client : clients_) {
      Send(client, &response);
    }
  }

  Frame &Next() {
    static size_t n = 0;
    if (n >= Log().size()) {
      n = 0;
    }
    return Log()[n++];
  }

  void OnReceive(uv_buf_t request, Addr addr) {
    clients_.emplace(addr);

    uv_buf_t response = request;
    for (const auto &client : clients_) {
      if (client != addr) {
        Send(client, &response);
      }
    }
  }

  Timer timer_;
  std::set<Addr> clients_;
};

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ReadLog(FLAGS_logfile);
  CHECK(Log().size() > 0);

  GameServer server;
  return server.ListenAndServe("127.0.0.1", 3389);
}
