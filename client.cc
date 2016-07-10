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

#include <functional>
#include <string>
#include <vector>

#pragma pack(push, 1)
struct State {
  float position[3];
  float orientation[4];
  uint32_t interacting;
};
#pragma pack(pop)

typedef std::vector<State> Frame;
typedef std::vector<Frame> Frames;

DEFINE_int32(cubes_count, 901, "Cubes count per frame");
DEFINE_string(logfile, "models.log", "Path to log file");

static Frames &Log() {
  static Frames log;
  return log;
}

static void ReadLog(std::string filename) {
  FILE *f = nullptr;
  CHECK(f = fopen(filename.c_str(), "r"));
  Log().clear();
  for (int frame = 1; frame; frame++) {
    std::vector<State> cubes;
    for (int i = 0; i < FLAGS_cubes_count; i++) {
      State s;
      if (fread(&s, sizeof(State), 1, f) != 1) {
        break;
      }
      cubes.push_back(s);
    }
    if (feof(f)) {
      break;
    }
    Log().push_back(cubes);
  }
  fclose(f);
}

class Loop {
  friend class Timer;

public:
  Loop() { CHECK(uv_loop_init(&loop_) == 0); }
  ~Loop() { uv_loop_close(&loop_); }

  int Run() { return uv_run(&loop_, UV_RUN_DEFAULT); }
  void Stop() { uv_stop(&loop_); }

private:
  uv_loop_t loop_;
};

class Timer {
public:
  Timer(Loop *loop, std::function<void(Timer *)> callback, uint64_t repeat)
      : callback_(std::move(callback)) {
    timer_.data = this;
    CHECK(uv_timer_init(&loop->loop_, &timer_) == 0);
    CHECK(uv_timer_start(&timer_, Timer::Wrapper, 1, repeat) == 0);
  }
  ~Timer() { uv_timer_stop(&timer_); }

private:
  static void Wrapper(uv_timer_t *handle) {
    ((Timer *)handle->data)->callback_((Timer *)handle->data);
  }
  std::function<void(Timer *)> callback_;

  uv_timer_t timer_;
};

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ReadLog(FLAGS_logfile);
  CHECK(Log().size() > 0);

  Loop loop;
  Timer frame_timer(
      &loop,
      [&](Timer *t) {
        static size_t frameno = 0;
        if (frameno >= Log().size()) {
          loop.Stop();
          return;
        }

        printf("Frame #%lu\n", frameno);

        auto cube = Log()[frameno][FLAGS_cubes_count - 1];
        printf("  Cube [(%f, %f %f), (%f, %f %f, %f), %s]\n", cube.position[0],
               cube.position[1], cube.position[2], cube.orientation[0],
               cube.orientation[1], cube.orientation[2], cube.orientation[3],
               (cube.interacting) ? "true" : "false");

        frameno++;
      },
      16);

  return loop.Run();
}