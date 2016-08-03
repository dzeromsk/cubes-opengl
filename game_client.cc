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

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <functional>
#include <deque>

#include "base/loop.h"
#include "base/idle.h"
#include "base/timer.h"
#include "base/window.h"
#include "net/udp.h"
#include "state.h"
#include "shader.h"
#include "program.h"
#include "model.h"

#include "game_client.h"

DECLARE_string(server_addr);
DECLARE_int32(server_port);

DECLARE_int32(width);
DECLARE_int32(height);

Client::Client(Loop &loop, Window &window, Model &model)
    : loop_(loop), window_(window), model_(model), socket_(loop),
      debug_socket_(loop), debug_enabled_(false), width_(FLAGS_width),
      height_(FLAGS_height), seq_(0), player_id_(1) {
  view_ = glm::vec3(0.f, 15.f, 25.f);
}

int Client::ConnectAndRun(const char *server_ip, int port) {
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

  socket_.OnAlloc([&](size_t suggested_size, uv_buf_t *buf) {
    static char slab[65536];
    *buf = uv_buf_init(slab, sizeof(slab));
  });

  socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                        unsigned flags) { OnReceive(buf, addr); });

  debug_socket_.OnAlloc([&](size_t suggested_size, uv_buf_t *buf) {
    static char slab[65536];
    *buf = uv_buf_init(slab, sizeof(slab));
  });

  debug_socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                              unsigned flags) { OnDebugReceive(buf, addr); });

  // TODO(dzeromsk): Send input to server at 30fps
  Timer input(&loop_,
              [&] {
                window_.Poll();
                if (window_.ShouldClose()) {
                  loop_.Stop();
                }
              },
              32);
  Timer render(&loop_, [&] { OnFrame(); }, 16);

  CHECK(uv_ip4_addr(server_ip, port, &server_addr_) == 0);

  Connect(socket_, server_addr_);

  return loop_.Run();
}

void Client::Connect(UDP &socket, const struct sockaddr_in &addr) {
  socket.Listen();
  uv_buf_t buf = { 0 };
  buf.base = (char *)"HELO";
  buf.len = 4;
  socket.Send(&buf, 1, (const sockaddr *)&addr);
}

void Client::Send(const char *command) {
  uv_buf_t buf = { 0 };
  buf.base = (char*)command;
  buf.len = strlen(command);
  socket_.Send(&buf, 1, (const sockaddr *)&server_addr_);
}

void Client::OnKey(int key) {
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

void Client::OnKeyPress(int key) {
  switch (key) {
  case GLFW_KEY_1:
    view_ = glm::vec3(0.f, 15.f, 25.f);
    break;
  case GLFW_KEY_2:
    view_ = glm::vec3(0.f, 50.f, 40.f);
    break;
  case GLFW_KEY_F11:
    window_.ToggleFullscreen();
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

void Client::OnFrame() {
  Frame &frame = Next();

  glm::mat4 projection =
      glm::perspective(45.0f, (GLfloat)width_ / (GLfloat)height_, 1.0f, 100.0f);

  glm::mat4 view = glm::lookAt(view_, glm::vec3(0), glm::vec3(0.0f, 1.0f, 0.0f));
  if (frame.size() >= player_id_)
    view = glm::lookAt(frame[player_id_].position + view_,
                       frame[player_id_].position, glm::vec3(0.0f, 1.0f, 0.0f));

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glBlendFunc(GL_ONE, GL_SRC_ALPHA);
  glDisable(GL_BLEND);

  model_.Draw(frame, view, projection);

  if (debug_enabled_) {
    DrawDebug(view, projection);
  }

  window_.Swap();
}

Frame &Client::Next() {
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
      uint32_t y_seq = s_.front();
      float a = 1 - ((y_seq - seq_) / 6.0f);
      frame_ = mix(x_, y, a);
      if (q_.size() > 0 && seq_ >= y_seq) {
        x_ = y;
        q_.pop_front();
        s_.pop_front();
        printf("!");

        // TODO(dzeromsk): Refactor!
        // We sometimes slip a frame on a client in comaprision to
        // server so here we compensate... :/
        if (q_.size() > 2) {
          Frame &y = q_.front();
          seq_ = y_seq;
          q_.pop_front();
          s_.pop_front();
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

Frame Client::mix(Frame &a, Frame &b, float step) {
  if (a.size() == b.size()) {
    size_t size = a.size();

    Frame m(size);
    for (size_t i = 0; i < size; ++i) {
      m[i].position = glm::mix(a[i].position, b[i].position, step);
      m[i].orientation = glm::slerp(a[i].orientation, b[i].orientation, step);
      m[i].interacting = a[i].interacting;
      m[i].scale = a[i].scale;
    }

    return m;
  } else {
    return a;
  }
}

void Client::OnReceive(uv_buf_t request, const struct sockaddr *addr) {
  Packet *p = (Packet *)request.base;

  static bool first_frame = true;
  if (first_frame) {
    seq_ = p->seq;
    player_id_ = p->size - 1;
    first_frame = false;
  }

  s_.emplace_back(p->seq);

  size_t q_size = p->size * sizeof(QState);
  CHECK(q_size < request.len);

  q_.emplace_back((QState *)p->data, (QState *)(p->data + q_size));
}

void Client::DrawDebug(const glm::mat4 &view, const glm::mat4 &projection) {
  glEnable(GL_BLEND);
  model_.Draw(debug_frame_, view, projection);
  glDisable(GL_BLEND);
}

void Client::OnDebugReceive(uv_buf_t request, const struct sockaddr *addr) {
  debug_frame_.assign((State *)request.base,
                      (State *)(request.base + request.len));
}
