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

#define container_of(ptr, type, member)                                        \
  ((type *)((char *)(ptr)-offsetof(type, member)))

inline uint32_t quantize(float x, int max_bits) {
  return x * ((1 << max_bits) - 1) + 0.5;
}

inline float dequantize(uint32_t x, int max_bits) {
  return float(x) / ((1 << max_bits) - 1);
}

inline float bound(float x, float min, float max) {
  return (x - min) / (max - min);
}

inline float unbound(float x, float min, float max) {
  return x * (max - min) + min;
}

#pragma pack(push, 1)
struct State {
  glm::vec3 position;
  glm::vec4 orientation;
  uint32_t interacting;
};
struct QState {
  int orientation_largest;
  int orientation[3];
  int position[3];
  int interacting;

  QState(const State &s);
};
#pragma pack(pop)

typedef std::vector<State> Frame;
typedef std::vector<Frame> Frames;

typedef std::vector<QState> QFrame;

QState::QState(const State &s) {
  position[0] = quantize(bound(s.position[0], -64, 64), 16);
  position[1] = quantize(bound(s.position[1], -1, 31), 14);
  position[2] = quantize(bound(s.position[2], -64, 64), 16);

  // interacting = !!s.interacting;
  interacting = s.interacting;

  // orientation smallest three method

  // find largest dimension
  float max = 0.0f;
  uint8_t maxno = 0;
  for (int i = 0; i < 4; i++) {
    float v = fabs(s.orientation[i]);
    if (v > max) {
      max = v;
      maxno = i;
    }
  }
  orientation_largest = maxno;

  // save remaining dimensions
  glm::vec3 abc(0.0f);
  for (int i = 0, j = 0; i < 4; i++) {
    if (i != maxno) {
      abc[j++] = s.orientation[i];
    }
  }

  // remeber to handle sign
  if (s.orientation[maxno] < 0) {
    abc *= -1;
  }

  float minimum = -1.0f / 1.414214f; // 1.0f / sqrt(2)
  float maximum = +1.0f / 1.414214f;

  for (int i = 0; i < 3; i++) {
    orientation[i] = quantize(bound(abc[i], minimum, maximum), 7);
  }
}

DEFINE_int32(cubes_count, 901, "Cubes count per frame");
DEFINE_string(logfile, "models.log", "Path to log file");

DEFINE_string(server_addr, "127.0.0.1", "Server ip address");
DEFINE_int32(server_port, 3389, "Server port");

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
  UDPServer(Loop &loop) : loop_(loop), socket_(loop_) {}

  typedef std::function<void(const uv_buf_t, const Addr)> ReceiveFunc;

  UDPServer(Loop &loop, ReceiveFunc on_receive)
      : loop_(loop), socket_(loop), on_receive_(std::move(on_receive)) {
    socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                          unsigned flags) { on_receive_(buf, addr); });
  }

  void OnReceive(ReceiveFunc on_receive) {
    on_receive_ = on_receive;
    socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                          unsigned flags) { on_receive_(buf, addr); });
  }

  void Listen(const char *ip, int port) { socket_.Listen(ip, port); }

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
  Loop &loop_;
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

class GameServer {
public:
  GameServer() : server_(loop_), debug_server_(loop_), timer_(loop_), seq_(0) {
    server_.OnReceive(
        [&](const uv_buf_t buf, const Addr addr) { OnReceive(buf, addr); });

    debug_server_.OnReceive([&](const uv_buf_t buf, const Addr addr) {
      OnDebugReceive(buf, addr);
    });

    timer_.Start(
        [&] {
          OnTick();
          OnDebugTick();

        },
        16);
  }

  int ListenAndServe(const char *ip, int port) {
    debug_server_.Listen(ip, port + 1);
    // we use the same loop so as an side effect ListenAndServe will start debug
    // server as well
    return server_.ListenAndServe(ip, port);
  }

private:
  Frame &Next(int n) { return Log()[n % Log().size()]; }

  void OnReceive(uv_buf_t request, Addr addr) {
    // for now just register the client
    clients_.emplace(addr);
  }

  void OnTick() {
    Frame &frame = Next(seq_);
    seq_++;

    if (!(seq_ % 6)) {
      qframe_.assign(begin(frame), end(frame));

      qframe_[0].interacting = seq_;
      uv_buf_t response = {(char *)qframe_.data(),
                           qframe_.size() * sizeof(QState)};
      for (const auto &client : clients_) {
        server_.Send(client, &response);
      }
    }
  }

  void OnDebugReceive(uv_buf_t request, Addr addr) {
    debug_clients_.emplace(addr);
  }

  void OnDebugTick() {
    Frame &frame = Next(seq_);

    uv_buf_t response = {(char *)frame.data(), frame.size() * sizeof(State)};
    for (const auto &client : debug_clients_) {
      debug_server_.Send(client, &response);
    }
  }

  int seq_;
  Loop loop_;
  UDPServer server_;
  Timer timer_;
  std::set<Addr> clients_;

  QFrame qframe_;

  UDPServer debug_server_;
  std::set<Addr> debug_clients_;
};

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ReadLog(FLAGS_logfile);
  CHECK(Log().size() > 0);

  GameServer server;
  return server.ListenAndServe(FLAGS_server_addr.c_str(), FLAGS_server_port);
}
