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

class UDP {
public:
  UDP(Loop &loop);

  ~UDP();

  void Listen(const char *ip, int port);

  void Listen();

  typedef std::function<void()> CloseFunc;
  typedef std::function<void(size_t, uv_buf_t *)> AllocFunc;
  typedef std::function<void(uv_buf_t, const struct sockaddr *, unsigned)>
      ReceiveFunc;

  void OnReceive(ReceiveFunc on_receive);

  void OnAlloc(AllocFunc on_alloc);

  int Send(const uv_buf_t bufs[], unsigned int nbufs,
           const struct sockaddr *addr);

private:
  static void CloseWrapper(uv_handle_t *handle);

  static void ReceiveWrapper(uv_udp_t *handle, ssize_t nread,
                             const uv_buf_t *buf, const struct sockaddr *addr,
                             unsigned flags);

  static void Alloc(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf);

  static void AllocWrapper(uv_handle_t *handle, size_t suggested_size,
                           uv_buf_t *buf);

  static void Send(uv_udp_send_t *req, int status);

  CloseFunc close_;
  AllocFunc alloc_;
  ReceiveFunc receive_;

#ifdef WIN32
  SOCKET socket_;
  Idle idle_;
#endif

#ifdef __linux__
  uv_udp_t socket_;
#endif
};