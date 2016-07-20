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

class Client {
public:
  Client(Loop &loop, Window &window, Model &model);

  int ConnectAndRun(const char *server_ip, int port);

private:
  void Connect(UDP &socket, const struct sockaddr_in &addr);

  void Send(const char *command);

  void OnKey(int key);

  void OnKeyPress(int key);

  void OnFrame();

  Frame &Next();

  Frame mix(Frame &a, Frame &b, float step);

  void OnReceive(uv_buf_t request, const struct sockaddr *addr);

  void DrawDebug(const glm::mat4 &view, const glm::mat4 &projection);

  void OnDebugReceive(uv_buf_t request, const struct sockaddr *addr);

  size_t seq_;
  Frame frame_;
  Frame x_;
  std::deque<Frame> q_;

  int width_;
  int height_;

  Loop &loop_;
  Model &model_;
  Window &window_;
  UDP socket_;
  struct sockaddr_in server_addr_;

  bool debug_enabled_;
  UDP debug_socket_;
  Frame debug_frame_;
  struct sockaddr_in debug_addr_;

  glm::mat4 view_;
};
