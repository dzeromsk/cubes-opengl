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

#pragma once

#define container_of(ptr, type, member)                                        \
  ((type *)((char *)(ptr)-offsetof(type, member)))

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

  void Listen() {
    CHECK(uv_udp_recv_start(&socket_, UDP::AllocWrapper, UDP::ReceiveWrapper) ==
          0);
  }

  typedef std::function<void()> CloseFunc;
  typedef std::function<void(size_t, uv_buf_t *)> AllocFunc;
  typedef std::function<void(uv_buf_t, const struct sockaddr *, unsigned)>
      ReceiveFunc;

  void OnReceive(ReceiveFunc on_receive) { receive_ = std::move(on_receive); }

  void OnAlloc(AllocFunc on_alloc) { alloc_ = std::move(on_alloc); }

  int Send(const uv_buf_t bufs[], unsigned int nbufs,
           const struct sockaddr *addr) {
    udp_send_ctx_t *req = (udp_send_ctx_t *)malloc(sizeof(*req));
    return uv_udp_send(&(req->req), &socket_, bufs, nbufs, addr, UDP::Send);
  }

private:
  static void CloseWrapper(uv_handle_t *handle) {
    ((UDP *)handle->data)->close_();
  }

  static void ReceiveWrapper(uv_udp_t *handle, ssize_t nread,
                             const uv_buf_t *buf, const struct sockaddr *addr,
                             unsigned flags) {
    if (nread > 0 && addr != nullptr) {
      uv_buf_t data = { 0 };
	  data.base = buf->base;
      data.len = size_t(nread);
      ((UDP *)handle->data)->receive_(data, addr, flags);
    }
  }

  static void Alloc(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf) {
    static char slab[65536];
    *buf = uv_buf_init(slab, sizeof(slab));
  }

  static void AllocWrapper(uv_handle_t *handle, size_t suggested_size,
                           uv_buf_t *buf) {
    ((UDP *)handle->data)->alloc_(suggested_size, buf);
  }

  static void Send(uv_udp_send_t *req, int status) {
    udp_send_ctx_t *ctx = container_of(req, udp_send_ctx_t, req);
    free(ctx);
  }

  CloseFunc close_;
  AllocFunc alloc_;
  ReceiveFunc receive_;

  uv_udp_t socket_;
};