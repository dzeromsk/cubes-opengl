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

#pragma pack(push, 1)
class State;
class QState;

struct State {
  glm::vec3 position;
  glm::quat orientation;
  uint32_t interacting;

  State() = default;
  State(const QState &qs);
};

struct QState {
  uint32_t orientation_largest : 2;
  uint32_t orientation_a : 7;
  uint32_t orientation_b : 7;
  uint32_t orientation_c : 7;
  uint32_t position_x : 12;
  uint32_t position_y : 8;
  uint32_t position_z : 12;
  uint32_t interacting;

  QState(const State &s);
};

struct Packet {
  uint32_t seq;
  uint32_t size;
  uint8_t  data[0];
};
#pragma pack(pop)

typedef std::vector<State> Frame;
typedef std::vector<Frame> Frames;
typedef std::vector<QState> QFrame;

inline uint32_t quantize(float x, int max_bits) {
  return x * ((1 << max_bits) - 1) + 0.5;
}

inline float dequantize(uint32_t x, int max_bits) {
  return float(x) / ((1 << max_bits) - 1);
}

inline float bound(float x, float min, float max) {
  return (x - min) / (max - min);
}

inline float unbound(float x, float min, float max) {
  return x * (max - min) + min;
}

inline State::State(const QState &qs) {
  position[0] = unbound(dequantize(qs.position_x, 12), -64, 64);
  position[1] = unbound(dequantize(qs.position_y, 8), -1, 31);
  position[2] = unbound(dequantize(qs.position_z, 12), -64, 64);

  // interacting = !!qs.interacting;
  interacting = qs.interacting;

  // smallest three method

  float minimum = -1.0f / 1.414214f; // 1.0f / sqrt(2)
  float maximum = +1.0f / 1.414214f;

  glm::vec3 abc;
  abc[0] = unbound(dequantize(qs.orientation_a, 7), minimum, maximum);
  abc[1] = unbound(dequantize(qs.orientation_b, 7), minimum, maximum);
  abc[2] = unbound(dequantize(qs.orientation_c, 7), minimum, maximum);

  for (size_t i = 0, j = 0; i < 4; i++) {
    if (i == qs.orientation_largest) {
      orientation[i] =
          sqrtf(1 - abc[0] * abc[0] - abc[1] * abc[1] - abc[2] * abc[2]);
    } else {
      orientation[i] = abc[j++];
    }
  }
}

inline QState::QState(const State &s) {
  position_x = quantize(bound(s.position[0], -64, 64), 12);
  position_y = quantize(bound(s.position[1], -1, 31), 8);
  position_z = quantize(bound(s.position[2], -64, 64), 12);

  // interacting = !!s.interacting;
  interacting = s.interacting;

  // orientation smallest three method

  // find largest dimension
  float max = 0.0f;
  uint8_t maxno = 0;
  for (size_t i = 0; i < 4; i++) {
    float v = fabs(s.orientation[i]);
    if (v > max) {
      max = v;
      maxno = i;
    }
  }
  orientation_largest = maxno;

  // save remaining dimensions
  glm::vec3 abc(0.0f);
  for (size_t i = 0, j = 0; i < 4; i++) {
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

  orientation_a = quantize(bound(abc[0], minimum, maximum), 7);
  orientation_b = quantize(bound(abc[1], minimum, maximum), 7);
  orientation_c = quantize(bound(abc[2], minimum, maximum), 7);
}