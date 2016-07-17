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
#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <glog/logging.h>
#include <uv.h>

#include <math.h>

uint32_t quantize(float x, int max_bits) {
  return x * ((1 << max_bits) - 1) + 0.5;
}

float dequantize(uint32_t x, int max_bits) {
  return float(x) / ((1 << max_bits) - 1);
}

float bound(float x, float min, float max) { return (x - min) / (max - min); }

float unbound(float x, float min, float max) { return x * (max - min) + min; }

struct QState;

#pragma pack(push, 1)
struct State {
  float position[3];
  float orientation[4];
  uint32_t interacting;

  State(const QState &qs);
  State() = default;
};

struct QState {
  int orientation_largest;
  int orientation[3];
  int position[3];
  int interacting;

  QState(const State &s);
};
#pragma pack(pop)

State::State(const QState &qs) {
  position[0] = unbound(dequantize(qs.position[0], 16), -64, 64);
  position[1] = unbound(dequantize(qs.position[1], 14), -1, 31);
  position[2] = unbound(dequantize(qs.position[2], 16), -64, 64);

  interacting = !!qs.interacting;

  // smallest three method

  float minimum = -1.0f / 1.414214f; // 1.0f / sqrt(2)
  float maximum = +1.0f / 1.414214f;

  glm::vec3 abc;
  for (int i = 0; i < 3; i++) {
    abc[i] = unbound(dequantize(qs.orientation[i], 9), minimum, maximum);
  }

  for (int i = 0, j = 0; i < 4; i++) {
    if (i == qs.orientation_largest) {
      orientation[i] =
          sqrtf(1 - abc[0] * abc[0] - abc[1] * abc[1] - abc[2] * abc[2]);
    } else {
      orientation[i] = abc[j++];
    }
  }
}

QState::QState(const State &s) {
  position[0] = quantize(bound(s.position[0], -64, 64), 16);
  position[1] = quantize(bound(s.position[1], -1, 31), 14);
  position[2] = quantize(bound(s.position[2], -64, 64), 16);

  interacting = !!s.interacting;

  // orientation smallest three method

  // find largest dimension
  float max = 0.0f;
  uint8_t maxno = 0;
  for (int i = 0; i < 4; i++) {
    float v = fabs(s.orientation[i]);
    if (v > max) {
      max = v;
      maxno = i;
    }
  }
  orientation_largest = maxno;

  // save remaining dimensions
  glm::vec3 abc(0.0f);
  for (int i = 0, j = 0; i < 4; i++) {
    if (i != maxno) {
      abc[j++] = s.orientation[i];
    }
  }

  // remeber to handle sign
  if (s.orientation[maxno] < 0) {
    abc *= -1;
  }

  float minimum = -1.0f / 1.414214f; // 1.0f / sqrt(2)
  float maximum = +1.0f / 1.414214f;

  for (int i = 0; i < 3; i++) {
    orientation[i] = quantize(bound(abc[i], minimum, maximum), 9);
  }
}

typedef std::vector<State> Frame;
typedef std::vector<Frame> Frames;

typedef std::vector<QState> QFrame;

static Frames &Log() {
  static Frames log;
  return log;
}

static void ReadLog(std::string filename) {
  FILE *f = nullptr;
  CHECK(f = fopen(filename.c_str(), "rb"));
  Log().clear();

  float max = 0, min = 0;
  for (int frame = 1; frame; frame++) {
    std::vector<State> cubes;
    for (int i = 0; i < 901; i++) {
      State s;
      if (fread(&s, sizeof(State), 1, f) != 1) {
        break;
      }
      cubes.push_back(s);
      // for (int i = 0; i < 3; i++)
      {
        int i = 2;
        if (s.position[i] > max) {
          max = s.position[i];
        }
        if (s.position[i] < min) {
          min = s.position[i];
        }
      }
    }
    if (feof(f)) {
      break;
    }
    Log().push_back(cubes);
  }
  fclose(f);
  printf("max %f, min %f\n", max, min);
}

bool valid(float aa, float bb) {
  float diff;
  float a = fabs(aa);
  float b = fabs(bb);

  if (a > b)
    diff = (a - b);
  else
    diff = (b - a);

  return diff < 0.1;
}

#define VALID(x, y)                                                            \
  if (!valid(x, y)) {                                                          \
    printf("{a: %f, b: %f}\n", x, y);                                          \
  }

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ReadLog("models.log");

  for (const auto &frame : Log()) {
    for (const auto &a : frame) {
      QState b(a);
      State c(b);

      VALID(a.position[0], c.position[0]);
      VALID(a.position[1], c.position[1]);
      VALID(a.position[2], c.position[2]);

      CHECK_EQ(a.interacting, c.interacting);

      VALID(a.orientation[0], c.orientation[0]);
      VALID(a.orientation[1], c.orientation[1]);
      VALID(a.orientation[2], c.orientation[2]);
      VALID(a.orientation[3], c.orientation[3]);
    }
  }

  return 0;
}
