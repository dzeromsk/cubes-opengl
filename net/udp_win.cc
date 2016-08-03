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

#include <glog/logging.h>
#include <uv.h>

#include <functional>

#include "base/loop.h"
#include "base/idle.h"
#include "net/udp.h"

UDP::UDP(Loop &loop) : idle_(loop) {
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  CHECK_NE(socket_, INVALID_SOCKET) << WSAGetLastError();
  DWORD yes = 1;
  CHECK(ioctlsocket(socket_, FIONBIO, &yes) != SOCKET_ERROR);
}

UDP::~UDP() { closesocket(socket_); }

void UDP::Listen(const char *ip, int port) {
  struct sockaddr_in addr;
  CHECK(uv_ip4_addr(ip, port, &addr) == 0);
  CHECK(bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) != INVALID_SOCKET)
      << WSAGetLastError();

  idle_.SetCallback([&] {
    ssize_t nread = 0;
    do {
      struct sockaddr_storage raddr = {0};
      size_t raddr_len = sizeof(raddr);
      uv_buf_t buf = {0};
      alloc_(65536, &buf);
      nread = recvfrom(socket_, buf.base, buf.len, 0, (struct sockaddr *)&raddr,
                       (int *)&raddr_len);
      if (nread > 0) {
        buf.len = size_t(nread);
        receive_(buf, (struct sockaddr *)&raddr, 0);
      }
    } while (nread != -1);
  });
}

void UDP::Listen() { Listen("0.0.0.0", 0); }

void UDP::OnReceive(ReceiveFunc on_receive) {
  receive_ = std::move(on_receive);
}

void UDP::OnAlloc(AllocFunc on_alloc) { alloc_ = std::move(on_alloc); }

int UDP::Send(const uv_buf_t bufs[], unsigned int nbufs,
              const struct sockaddr *addr) {
  return sendto(socket_, bufs[0].base, bufs[0].len, 0, (struct sockaddr *)addr,
                sizeof(struct sockaddr));
}