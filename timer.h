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

class Timer {
public:
  Timer(Loop &loop) {
    timer_.data = this;
    CHECK(uv_timer_init(&loop.loop_, &timer_) == 0);
  }

  Timer(Loop *loop, std::function<void(void)> callback, uint64_t repeat)
      : callback_(std::move(callback)) {
    timer_.data = this;
    CHECK(uv_timer_init(&loop->loop_, &timer_) == 0);
    CHECK(uv_timer_start(&timer_, Timer::Wrapper, 1, repeat) == 0);
  }

  ~Timer() { uv_timer_stop(&timer_); }

  void Start(std::function<void(void)> callback, uint64_t repeat) {
    callback_ = std::move(callback);
    CHECK(uv_timer_start(&timer_, Timer::Wrapper, 1, repeat) == 0);
  }

private:
  static void Wrapper(uv_timer_t *handle) {
    ((Timer *)handle->data)->callback_();
  }
  std::function<void(void)> callback_;

  uv_timer_t timer_;
};
