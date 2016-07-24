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

#include <functional>

#include "base/loop.h"
#include "base/timer.h"
#include "base/window.h"

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_IMPLEMENTATION
#define NK_GLFW_GL3_IMPLEMENTATION
#include "third_party/nuklear/nuklear.h"
#include "third_party/nuklear/nuklear_glfw_gl3.h"

#include "menu.h"

DECLARE_int32(width);
DECLARE_int32(height);
DECLARE_string(server_addr);

Menu &Menu::Default() {
  static Menu gui(Window::Default());
  return gui;
}
Menu::~Menu() { nk_glfw3_shutdown(); }

int Menu::Run() {
  window_.OnKey([&](int key, int action) {
    if (key == GLFW_KEY_F11 && action == GLFW_PRESS) {
      window_.ToggleFullscreen();
    }
  });

  timer_.Start(
      [&] {
        glClear(GL_COLOR_BUFFER_BIT);
        window_.Poll();
        status_ = Draw();
        if (status_) {
          loop_.Stop();
        }
        Render();
        window_.Swap();
      },
      32);

  loop_.Run();

  return status_;
}

Menu::Menu(Window &window)
    : window_(window), window_width_(FLAGS_width), window_height_(FLAGS_height),
      timer_(loop_), status_(0) {
  ctx_ = nk_glfw3_init(window.window_, NK_GLFW3_INSTALL_CALLBACKS);
  struct nk_font_atlas *atlas;
  nk_glfw3_font_stash_begin(&atlas);
  nk_glfw3_font_stash_end();
  window_.OnResize([&](int w, int h) {
    glViewport(0, 0, w, h);
    window_width_ = w;
    window_height_ = h;
  });
}

int Menu::Draw(int width, int height) {
  nk_glfw3_new_frame();

  struct nk_panel layout;
  if (nk_begin(ctx_, &layout, "Start game",
               nk_rect(window_width_ / 2 - width / 2,
                       window_height_ / 2 - height / 2, width, height),
               NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_TITLE)) {
    static char text[64] = "127.0.0.1";
    static int text_len = strlen(text);
    static const float ratio[] = {0.45, 0.55};

    nk_layout_row(ctx_, NK_DYNAMIC, 25, 2, ratio);
    nk_label(ctx_, "Server Address:", NK_TEXT_LEFT);
    nk_edit_string(ctx_, NK_EDIT_SIMPLE, text, &text_len, 64,
                   nk_filter_default);

    nk_layout_row_dynamic(ctx_, 30, 1);
    if (nk_button_label(ctx_, "Connect", NK_BUTTON_DEFAULT)) {
      std::string server_ip(text, text_len);
      struct sockaddr_in server_addr;
      if (uv_ip4_addr(server_ip.c_str(), 1, &server_addr) == 0) {
        FLAGS_server_addr = server_ip;
        return 1;
      }
    }

    nk_layout_row_dynamic(ctx_, 30, 1);
    nk_label(ctx_, "   - or -", NK_TEXT_CENTERED);

    nk_layout_row_dynamic(ctx_, 30, 1);
    if (nk_button_label(ctx_, "New game", NK_BUTTON_DEFAULT)) {
      return 2;
    }

    nk_end(ctx_);
  }
  return 0;
}

void Menu::Render() {
  nk_glfw3_render(NK_ANTI_ALIASING_ON, 512 * 1024, 128 * 1024);
}
