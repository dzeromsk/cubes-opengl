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

class Idle {
public:
  Idle(Loop &loop) { CHECK(uv_idle_init(&loop.loop_, &idle_) == 0); }

  ~Idle() {
    uv_idle_stop(&idle_);
    uv_close((uv_handle_t *)&idle_, nullptr);
  }

  void SetCallback(std::function<void(void)> callback) {
    callback_ = std::move(callback);
    idle_.data = this;
    CHECK(uv_idle_start(&idle_, Wrapper) == 0);
  }

private:
  static void Wrapper(uv_idle_t *handle) {
    ((Idle *)handle->data)->callback_();
  }

  uv_idle_t idle_;
  std::function<void(void)> callback_;
};
