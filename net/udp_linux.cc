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
#include "net/udp.h"

#define container_of(ptr, type, member)                                        \
  ((type *)((char *)(ptr)-offsetof(type, member)))

typedef struct {
  uv_udp_send_t req;
  char *data;
} udp_send_ctx_t;

UDP::UDP(Loop &loop) {
  CHECK(uv_udp_init(&loop.loop_, &socket_) == 0);
  socket_.data = this;
}

UDP::~UDP() { uv_udp_recv_stop(&socket_); }

void UDP::Listen(const char *ip, int port) {
  struct sockaddr_in addr;
  CHECK(uv_ip4_addr(ip, port, &addr) == 0);
  CHECK(uv_udp_bind(&socket_, (const struct sockaddr *)&addr, 0) == 0);
  CHECK(uv_udp_recv_start(&socket_, AllocWrapper, ReceiveWrapper) == 0);
}

void UDP::Listen() {
  CHECK(uv_udp_recv_start(&socket_, AllocWrapper, ReceiveWrapper) == 0);
}

void UDP::OnReceive(ReceiveFunc on_receive) {
  receive_ = std::move(on_receive);
}

void UDP::OnAlloc(AllocFunc on_alloc) { alloc_ = std::move(on_alloc); }

int UDP::Send(const uv_buf_t bufs[], unsigned int nbufs,
              const struct sockaddr *addr) {
  udp_send_ctx_t *req = (udp_send_ctx_t *)malloc(sizeof(*req));
  return uv_udp_send(&(req->req), &socket_, bufs, nbufs, addr, UDP::Send);
}

void UDP::CloseWrapper(uv_handle_t *handle) { ((UDP *)handle->data)->close_(); }

void UDP::ReceiveWrapper(uv_udp_t *handle, ssize_t nread, const uv_buf_t *buf,
                         const struct sockaddr *addr, unsigned flags) {
  if (nread > 0 && addr != nullptr) {
    uv_buf_t data;
    data.base = buf->base;
    data.len = size_t(nread);
    ((UDP *)handle->data)->receive_(data, addr, flags);
  }
}

void UDP::Alloc(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf) {
  static char slab[65536];
  *buf = uv_buf_init(slab, sizeof(slab));
}

void UDP::AllocWrapper(uv_handle_t *handle, size_t suggested_size,
                       uv_buf_t *buf) {
  ((UDP *)handle->data)->alloc_(suggested_size, buf);
}

void UDP::Send(uv_udp_send_t *req, int status) {
  udp_send_ctx_t *ctx = container_of(req, udp_send_ctx_t, req);
  free(ctx);
}