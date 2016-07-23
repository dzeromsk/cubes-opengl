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

class UDPServer {
public:
  UDPServer(Loop &loop) : loop_(loop), socket_(loop_) {
    socket_.OnAlloc([&](size_t suggested_size, uv_buf_t *buf) {
      static char slab[65536];
      *buf = uv_buf_init(slab, sizeof(slab));
    });
  }

  typedef std::function<void(const uv_buf_t, const Addr)> ReceiveFunc;

  UDPServer(Loop &loop, ReceiveFunc on_receive)
      : loop_(loop), socket_(loop), on_receive_(std::move(on_receive)) {
    socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                          unsigned flags) { on_receive_(buf, addr); });
    socket_.OnAlloc([&](size_t suggested_size, uv_buf_t *buf) {
      static char slab[65536];
      *buf = uv_buf_init(slab, sizeof(slab));
    });
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