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

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#include "third_party/nuklear/nuklear.h"
#include "third_party/nuklear/nuklear_glfw_gl3.h"

#define MAX_VERTEX_BUFFER 512 * 1024
#define MAX_ELEMENT_BUFFER 128 * 1024

class HUD {
public:
  HUD(GLFWwindow *window)
      : delay_(30), input_delay_(1), last_update_(0), frames_(0) {
    ctx_ = nk_glfw3_init(window, NK_GLFW3_INSTALL_CALLBACKS);
    struct nk_font_atlas *atlas;
    nk_glfw3_font_stash_begin(&atlas);
    nk_glfw3_font_stash_end();
  }

  void Draw() {
    GLfloat now = glfwGetTime();

    if (now - last_update_ >= 1.0) {
      snprintf(buffer_, sizeof(buffer_), "%d", frames_);
      fps_history_.append(frames_);
      frames_ = 0;
      last_update_ = now;
    }
    frames_++;

    nk_glfw3_new_frame();

    {
      struct nk_panel layout;
      if (nk_begin(ctx_, &layout, "Cubes", nk_rect(50, 50, 230, 230),
                   NK_WINDOW_BORDER | NK_WINDOW_MOVABLE |
                       NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE)) {

        nk_layout_row_dynamic(ctx_, 50, 1);
        if (nk_chart_begin(ctx_, NK_CHART_COLUMN, 15, 0, 1.5f)) {
          for (int i = -15; i < 0; ++i) {
            nk_chart_push(ctx_, (float)fps_history_[i] / 60);
          }
          nk_chart_end(ctx_);
        }

        static const float ratio[] = {60, 135};

        nk_layout_row(ctx_, NK_STATIC, 25, 2, ratio);
        nk_label(ctx_, "FPS:", NK_TEXT_LEFT);
        int len = strlen(buffer_);
        nk_edit_string(ctx_, NK_EDIT_SIMPLE | NK_EDIT_READ_ONLY, buffer_, &len,
                       128, nk_filter_default);

        nk_layout_row(ctx_, NK_STATIC, 25, 2, ratio);
        nk_label(ctx_, "Delay:", NK_TEXT_LEFT);
        nk_progress(ctx_, &delay_, 60, NK_MODIFIABLE);
        nk_layout_row(ctx_, NK_STATIC, 25, 2, ratio);
        nk_label(ctx_, "In Delay:", NK_TEXT_LEFT);
        nk_progress(ctx_, &input_delay_, 60, NK_MODIFIABLE);
      }
      nk_end(ctx_);
    }

    nk_glfw3_render(NK_ANTI_ALIASING_ON, MAX_VERTEX_BUFFER, MAX_ELEMENT_BUFFER);
  }

  size_t GetDelay() { return delay_; }
  size_t GetInputDelay() { return input_delay_; }

private:
  struct nk_context *ctx_;
  size_t delay_;
  size_t input_delay_;
  int frames_;
  GLfloat last_update_;
  CircularBuffer<int, 30> fps_history_;
  char buffer_[256];
};