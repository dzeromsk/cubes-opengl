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

DECLARE_string(server_addr);
DECLARE_int32(server_port);

class Client {
public:
  Client(Loop &loop, Window &window, Model &model)
      : loop_(loop), window_(window), model_(model), socket_(loop),
        debug_socket_(loop), debug_enabled_(false), width_(FLAGS_width),
        height_(FLAGS_height), seq_(0) {
    view_ = glm::lookAt(glm::vec3(0.f, 50.f, 40.f), glm::vec3(0, 0, 0),
                        glm::vec3(0.0f, 1.0f, 0.0f));
  }

  int ConnectAndRun(const char *server_ip, int port) {
    window_.OnResize([&](int w, int h) {
      glViewport(0, 0, w, h);
      width_ = w;
      height_ = h;
    });

    window_.OnKey([&](int key, int action) {
      if (action == GLFW_PRESS) {
        OnKeyPress(key);
      }
      OnKey(key);
    });

    socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                          unsigned flags) { OnReceive(buf, addr); });

    debug_socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                                unsigned flags) { OnDebugReceive(buf, addr); });

    // TODO(dzeromsk): Send input to server at 30fps
    Timer input(&loop_, [&] { window_.Poll(); }, 32);
    Timer render(&loop_, [&] { OnFrame(); }, 16);

    CHECK(uv_ip4_addr(server_ip, port, &server_addr_) == 0);

    Connect(socket_, server_addr_);

    return loop_.Run();
  }

private:
  void Connect(UDP &socket, const struct sockaddr_in &addr) {
    socket.Listen();
    uv_buf_t buf = {(char *)"HELO", 4};
    socket.Send(&buf, 1, (const sockaddr *)&addr);
  }

  void Send(const char *command) {
    uv_buf_t buf = {(char *)command, strlen(command)};
    socket_.Send(&buf, 1, (const sockaddr *)&server_addr_);
  }

  void OnKey(int key) {
    switch (key) {
    case GLFW_KEY_W:
      Send("w");
      break;
    case GLFW_KEY_S:
      Send("s");
      break;
    case GLFW_KEY_A:
      Send("a");
      break;
    case GLFW_KEY_D:
      Send("d");
      break;
    case GLFW_KEY_R:
      Send("r");
      break;
    case GLFW_KEY_ESCAPE:
    case GLFW_KEY_Q:
      Send("q");
      loop_.Stop();
      break;
    default:
      break;
    }
  }

  void OnKeyPress(int key) {
    switch (key) {
    case GLFW_KEY_1:
      view_ = glm::lookAt(glm::vec3(0.f, 15.f, 25.f), glm::vec3(0, 0, 0),
                          glm::vec3(0.0f, 1.0f, 0.0f));
      break;
    case GLFW_KEY_2:
      view_ = glm::lookAt(glm::vec3(0.f, 50.f, 40.f), glm::vec3(0, 0, 0),
                          glm::vec3(0.0f, 1.0f, 0.0f));
      break;
    case GLFW_KEY_F12:
      debug_enabled_ = !debug_enabled_;
      {
        static bool once = true;
        if (once) {
          CHECK(uv_ip4_addr(FLAGS_server_addr.c_str(), FLAGS_server_port + 1,
                            &debug_addr_) == 0);
          Connect(debug_socket_, debug_addr_);
          once = false;
        }
      }
      break;
    default:
      break;
    }
  }

  void OnFrame() {
    Frame &frame = Next();

    glm::mat4 projection = glm::perspective(
        45.0f, (GLfloat)width_ / (GLfloat)height_, 1.0f, 100.0f);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glBlendFunc(GL_ONE, GL_SRC_ALPHA);
    glDisable(GL_BLEND);

    // std::vector<glm::mat4> models;
    // for (const auto &cube : frame) {
    //   // TODO(dzeromsk): compute model matrix and set color in
    //   // shader

    //   glm::mat4 translate = glm::translate(glm::mat4(1.0), cube.position);
    //   glm::mat4 rotate = glm::mat4_cast(cube.orientation);

    //   models.push_back(translate * rotate);
    // }

    model_.Draw(frame, view_, projection);

    if (debug_enabled_) {
      DrawDebug(view_, projection);
    }

    window_.Swap();
  }

  Frame &Next() {
    static bool buffering = true;
    if (buffering) {
      if (q_.size() > 1) {
        buffering = false;
      }
      printf("@");
      return x_;
    } else {

      if (q_.size() > 0) {
        Frame y = q_.front();
        float a = 1 - ((y[0].interacting - seq_) / 6.0f);
        frame_ = mix(x_, y, a);
        if (q_.size() > 0 && seq_ >= y[0].interacting) {
          x_ = y;
          q_.pop_front();
          printf("!");

          // TODO(dzeromsk): Refactor!
          // We sometimes slip a frame on a client in comaprision to
          // server so here we compensate... :/
          if (q_.size() > 2) {
            Frame &y = q_.front();
            seq_ = y[0].interacting;
            q_.pop_front();
          }
        } else {
          printf(".");
        }

        seq_++;
        return frame_;
      } else {
        printf(":");
        return frame_;
      }
    }
  }

  Frame mix(Frame &a, Frame &b, float step) {
    if (a.size() == b.size()) {
      size_t size = a.size();

      Frame m(size);
      for (size_t i = 0; i < size; ++i) {
        m[i].position = glm::mix(a[i].position, b[i].position, step);
        m[i].orientation = glm::slerp(a[i].orientation, b[i].orientation, step);
        m[i].interacting = a[i].interacting && b[i].interacting;
      }

      return m;
    } else {
      return a;
    }
  }

  void OnReceive(uv_buf_t request, const struct sockaddr *addr) {
    static bool first_frame = true;
    if (first_frame) {
      seq_ = ((QState *)request.base)->interacting;
      first_frame = false;
    }

    q_.emplace_back((QState *)request.base,
                    (QState *)(request.base + request.len));
  }

  void DrawDebug(const glm::mat4 &view, const glm::mat4 &projection) {
    glEnable(GL_BLEND);
    model_.Draw(debug_frame_, view, projection);
    glDisable(GL_BLEND);
  }

  void OnDebugReceive(uv_buf_t request, const struct sockaddr *addr) {
    debug_frame_.assign((State *)request.base,
                        (State *)(request.base + request.len));
  }

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
