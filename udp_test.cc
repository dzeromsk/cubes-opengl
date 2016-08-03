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
#include <glog/logging.h>
#include <uv.h>

#include <functional>

#include "base/loop.h"

class Idle {
public:
  Idle(Loop &loop) { CHECK(uv_idle_init(&loop.loop_, &idle_) == 0); }

  ~Idle() {
    uv_idle_stop(&idle_);
    uv_close((uv_handle_t *)&idle_, nullptr);
  }

  void SetCallback(std::function<void(void)> callback) {
    callback_ = std::move(callback);
    idle_.data = this;
    CHECK(uv_idle_start(&idle_, Wrapper) == 0);
  }

private:
  static void Wrapper(uv_idle_t *handle) {
    ((Idle *)handle->data)->callback_();
  }

  uv_idle_t idle_;
  std::function<void(void)> callback_;
};

class UDP {
public:
  UDP(Loop &loop) : idle_(loop) {
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    CHECK_NE(socket_, INVALID_SOCKET) << WSAGetLastError();
    DWORD yes = 1;
    CHECK(ioctlsocket(socket_, FIONBIO, &yes) != SOCKET_ERROR);
  }

  ~UDP() { closesocket(socket_); }

  void Listen(const char *ip, int port) {
    struct sockaddr_in addr;
    CHECK(uv_ip4_addr(ip, port, &addr) == 0);
    CHECK(bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) !=
          INVALID_SOCKET)
        << WSAGetLastError();
    idle_.SetCallback([&] {
      ssize_t nread = 0;
      do {
        struct sockaddr_storage raddr = {0};
        size_t raddr_len = sizeof(raddr);
        uv_buf_t buf = {0};
        alloc_callback_(65536, &buf);
        nread = recvfrom(socket_, buf.base, buf.len, 0,
                         (struct sockaddr *)&raddr, (int *)&raddr_len);
        if (nread > 0) {
          buf.len = size_t(nread);
          receive_callback_(buf, (struct sockaddr *)&raddr, 0);
        }
      } while (nread != -1);
    });
  }

  void Listen() { Listen("0.0.0.0", 0); }

  typedef std::function<void(size_t, uv_buf_t *)> AllocFunc;
  typedef std::function<void(uv_buf_t, const struct sockaddr *, unsigned)>
      ReceiveFunc;

  void OnReceive(ReceiveFunc on_receive) {
    receive_callback_ = std::move(on_receive);
  }

  void OnAlloc(AllocFunc on_alloc) { alloc_callback_ = std::move(on_alloc); }

  int Send(const uv_buf_t bufs[], unsigned int nbufs,
           const struct sockaddr *addr) {
    return sendto(socket_, bufs[0].base, bufs[0].len, 0,
                  (struct sockaddr *)addr, sizeof(struct sockaddr));
  }

private:
  SOCKET socket_;
  Idle idle_;
  AllocFunc alloc_callback_;
  ReceiveFunc receive_callback_;
};

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  Loop loop;

  UDP udp(loop);
  udp.Listen("127.0.0.1", 33389);

  udp.OnAlloc([&](size_t suggested_size, uv_buf_t *buf) {
    static char slab[65536];
    *buf = uv_buf_init(slab, sizeof(slab));
  });

  udp.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr, unsigned flags) {
    printf("OnRecive()\n");
    uv_buf_t res = {0};
    res.base = (char *)"PONG\n";
    res.len = 5;
    udp.Send(&res, 1, (const sockaddr *)addr);
  });

  return loop.Run();
}
