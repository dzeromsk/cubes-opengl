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
    CHECK(uv_udp_recv_start(&socket_, UDP::AllocWrapper, UDP::ReceiveWrapper) ==
          0);
  }

  typedef std::function<void(uv_handle_t *, size_t, uv_buf_t *)> AllocFunc;
  typedef std::function<void(uv_handle_t *)> CloseFunc;
  typedef std::function<void(uv_udp_t *, ssize_t, const uv_buf_t *,
                             const struct sockaddr *, unsigned)>
      ReceiveFunc;
  typedef std::function<void(uv_udp_send_t *, int)> SendFunc;

  void OnAlloc(AllocFunc on_alloc) { alloc_ = std::move(on_alloc); }

  void OnReceive(ReceiveFunc on_receive) { receive_ = std::move(on_receive); }

  int Send(const uv_buf_t bufs[], unsigned int nbufs,
           const struct sockaddr *addr) {
    udp_send_ctx_t *req = (udp_send_ctx_t *)malloc(sizeof(*req));
    uv_udp_send(&(req->req), &socket_, bufs, nbufs, addr, UDP::SendWrapper);
  }

private:
  static void AllocWrapper(uv_handle_t *handle, size_t suggested_size,
                           uv_buf_t *buf) {
    ((UDP *)handle->data)->alloc_(handle, suggested_size, buf);
  }

  static void CloseWrapper(uv_handle_t *handle) {
    ((UDP *)handle->data)->close_(handle);
  }

  static void ReceiveWrapper(uv_udp_t *handle, ssize_t nread,
                             const uv_buf_t *buf, const struct sockaddr *addr,
                             unsigned flags) {
    if (nread > 0 && addr != nullptr) {
      ((UDP *)handle->data)->receive_(handle, nread, buf, addr, flags);
    }
  }

  static void SendWrapper(uv_udp_send_t *req, int status) {
    udp_send_ctx_t *ctx = container_of(req, udp_send_ctx_t, req);
    free(ctx);
  }

  AllocFunc alloc_;
  CloseFunc close_;
  ReceiveFunc receive_;
  SendFunc send_;

  uv_udp_t socket_;
};

class UDPServer {
public:
  typedef std::function<void(const uv_buf_t *, ssize_t,
                             const struct sockaddr *)>
      ReceiveFunc;
  UDPServer(ReceiveFunc on_receive)
      : socket_(loop_), on_receive_(std::move(on_receive)) {
    socket_.OnAlloc(
        [&](uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf) {
          CHECK(handle);
          CHECK(suggested_size <= 65536);
          *buf = OnAlloc(suggested_size);
        });

    socket_.OnReceive([&](uv_udp_t *handle, ssize_t nread, const uv_buf_t *buf,
                          const struct sockaddr *addr,
                          unsigned flags) { on_receive_(buf, nread, addr); });
  }

  int ListenAndServe(const char *ip, int port) {
    socket_.Listen(ip, port);
    return loop_.Run();
  }

  int Send(const struct sockaddr *addr, const uv_buf_t *buf) {
    socket_.Send(buf, 1, addr);
  }

private:
  uv_buf_t OnAlloc(size_t suggested_size) {
    static char slab[65536];
    return uv_buf_init(slab, sizeof(slab));
  }

  Loop loop_;
  UDP socket_;
  ReceiveFunc on_receive_;
};

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ReadLog(FLAGS_logfile);
  CHECK(Log().size() > 0);

  UDPServer server(
      [&](const uv_buf_t *buf, ssize_t x, const struct sockaddr *addr) {
        printf("Got message! (%ld, %p)\n", x, addr);
        printf("- %s", buf->base);
        uv_buf_t response = uv_buf_init((char *)"PONG!\n", 6);

        server.Send(addr, &response);
      });

  return server.ListenAndServe("127.0.0.1", 3389);
}