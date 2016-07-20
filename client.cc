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
#include <glm/gtc/type_ptr.hpp>

#include <cmath>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "loop.h"
#include "timer.h"
#include "udp.h"
#include "state.h"
#include "window.h"

DEFINE_string(server_addr, "127.0.0.1", "Server ip address");
DEFINE_int32(server_port, 3389, "Server port");

static Frames &Log() {
  static Frames log;
  return log;
}

namespace render {

#define GLSL(src) "#version 330 core\n" #src

// clang-format off
const char *kVertexSource = GLSL(
  in vec3 position;
  in vec3 normal;

  // state
  in vec3 pos; // instance
  in vec4 orie; // instance

  out vec3 FragPos;
  out vec3 Normal;

  uniform mat4 view;
  uniform mat4 projection;
  uniform float alpha;

  void mat4_from_quat(out mat4 m, in vec4 q) {
    float xx = q.x * q.x;
    float yy = q.y * q.y;
    float zz = q.z * q.z;
    float xz = q.x * q.z;
    float xy = q.x * q.y;
    float yz = q.y * q.z;
    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;

    m[0][0] = 1 - 2 * (yy + zz);
    m[0][1] = 2 * (xy + wz);
    m[0][2] = 2 * (xz - wy);
    m[0][3] = 0;

    m[1][0] = 2 * (xy - wz);
    m[1][1] = 1 - 2 * (xx + zz);
    m[1][2] = 2 * (yz + wx);
    m[1][3] = 0;

    m[2][0] = 2 * (xz + wy);
    m[2][1] = 2 * (yz - wx);
    m[2][2] = 1 - 2 * (xx + yy);
    m[2][3] = 0;

    m[3][0] = 0;
    m[3][1] = 0;
    m[3][2] = 0;
    m[3][3] = 1;
  }

  void main() {
    mat4 model;
    mat4_from_quat(model, orie);
    model[3] = vec4(pos, 1.0f);
    //gl_Position = projection * view * model * vec4(position, 1.0f);
    gl_Position = projection * view * model * vec4(position, alpha);
    FragPos = vec3(model * vec4(position, 1.0f));
    Normal = vec3(model * vec4(normal, 0));
  }
);
// clang-format on

// clang-format off
const char *kFragmentSource = GLSL(
  in vec3 Normal;
  in vec3 FragPos;

  out vec4 color;

  uniform vec3 light_position;
  uniform vec3 light_color;
  uniform vec3 object_color;

  void main() {
    // Ambient
    float ambient_strength = 0.1f;
    vec3 ambient = ambient_strength * light_color;

    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 light_direction = normalize(light_position - FragPos);
    float diff = max(dot(norm, light_direction), 0.0);
    // float diff = max(dot(norm, vec3(0.0, 1.0, 0.0)), 0.0);
    vec3 diffuse = diff * light_color;

    vec3 result = (ambient + diffuse) * object_color;
    color = vec4(result, 1.0f);
    // color = vec4(norm*.5f + .5f, 1.0f);
  }
);
// clang-format on

// clang-format off
const GLfloat kVertices[] = {
  -0.5f, -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f,
  0.5f,  -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f,
  0.5f,  0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f,
  0.5f,  0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f,
  -0.5f, 0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f,
  -0.5f, -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f,

  -0.5f, -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,
  0.5f,  -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,
  0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  -0.5f, 0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  -0.5f, -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,

  -0.5f, 0.5f,  0.5f,  -1.0f, 0.0f,  0.0f,
  -0.5f, 0.5f,  -0.5f, -1.0f, 0.0f,  0.0f,
  -0.5f, -0.5f, -0.5f, -1.0f, 0.0f,  0.0f,
  -0.5f, -0.5f, -0.5f, -1.0f, 0.0f,  0.0f,
  -0.5f, -0.5f, 0.5f,  -1.0f, 0.0f,  0.0f,
  -0.5f, 0.5f,  0.5f,  -1.0f, 0.0f,  0.0f,

  0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
  0.5f,  0.5f,  -0.5f, 1.0f,  0.0f,  0.0f,
  0.5f,  -0.5f, -0.5f, 1.0f,  0.0f,  0.0f,
  0.5f,  -0.5f, -0.5f, 1.0f,  0.0f,  0.0f,
  0.5f,  -0.5f, 0.5f,  1.0f,  0.0f,  0.0f,
  0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,

  -0.5f, -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f,
  0.5f,  -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f,
  0.5f,  -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,
  0.5f,  -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,
  -0.5f, -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,
  -0.5f, -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f,

  -0.5f, 0.5f,  -0.5f, 0.0f,  1.0f,  0.0f,
  0.5f,  0.5f,  -0.5f, 0.0f,  1.0f,  0.0f,
  0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
  0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f, 0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f, 0.5f,  -0.5f, 0.0f,  1.0f,  0.0f
};
// clang-format on

const size_t kVerticesSize = sizeof(kVertices);

struct Shader {
  GLuint shader;

  Shader(const char *source, int type) {
    shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    Verify();
  }
  ~Shader() { glDeleteShader(shader); }

  void Verify() {
    GLint status = 0;
    GLint logsz = 0;
    std::string log;

    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logsz);

    log.resize(logsz);
    glGetShaderInfoLog(shader, logsz, &logsz, (char *)log.data());

    CHECK(status == GL_TRUE) << "Build failed: " << log;
  }
};

struct Program {
  GLuint program;

  Program(const char *vertex, const char *fragment) {
    Shader v(vertex, GL_VERTEX_SHADER);
    Shader f(fragment, GL_FRAGMENT_SHADER);

    program = glCreateProgram();
    glAttachShader(program, v.shader);
    glAttachShader(program, f.shader);
    glLinkProgram(program);

    Verify();
  }
  ~Program() { glDeleteProgram(program); }

  void Verify() {
    GLint status = 0;
    GLint logsz = 0;
    std::string log;

    glGetProgramiv(program, GL_LINK_STATUS, &status);
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logsz);

    log.resize(logsz);
    glGetProgramInfoLog(program, logsz, &logsz, (char *)log.data());

    CHECK(status == GL_TRUE) << "Link failed: " << log;
  }

  void Use() { glUseProgram(program); }
};

struct Model {
  Program program;

  GLint view;
  GLint projection;
  GLint object_color;
  GLint light_color;
  GLint light_position;

  // state
  GLint pos;
  GLint orie;
  GLint flags;

  GLint position;
  GLint normal;
  GLint alpha;

  GLuint VBO[2];
  GLuint VAO;

  size_t vertices_size;
  size_t frame_size;

  Model()
      : program(kVertexSource, kFragmentSource), vertices_size(kVerticesSize),
        frame_size(0) {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(2, VBO);
    Locations();
    Attrib();
    WriteVertices(kVertices, kVerticesSize);
  }

  ~Model() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(2, VBO);
  }

  void Locations() {
#define GET(name)                                                              \
  name = glGetUniformLocation(program.program, #name);                         \
  CHECK(name > -1) << #name;
    GET(view);
    GET(projection);
    GET(object_color);
    GET(light_color);
    GET(light_position);
    GET(alpha);
#undef GET
#define GET(name)                                                              \
  name = glGetAttribLocation(program.program, #name);                          \
  CHECK(name > -1) << #name;
    GET(position)
    GET(normal)
    GET(pos);
    GET(orie);
// GET(flags);
#undef GET
  }

  void Attrib() {
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);

    // Position attribute
    glVertexAttribPointer(position, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat),
                          (GLvoid *)0);
    glEnableVertexAttribArray(position);

    // Normal attribute
    glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat),
                          (GLvoid *)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(normal);

    glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);

    glVertexAttribPointer(pos, 3, GL_FLOAT, GL_FALSE, sizeof(State),
                          (GLvoid *)0);
    glEnableVertexAttribArray(pos);
    glVertexAttribDivisor(pos, 1);

    glVertexAttribPointer(orie, 4, GL_FLOAT, GL_FALSE, sizeof(State),
                          (GLvoid *)(sizeof(glm::vec3)));
    glEnableVertexAttribArray(orie);
    glVertexAttribDivisor(orie, 1);

    // glVertexAttribPointer(flags, 1, GL_UNSIGNED_INT, GL_FALSE, sizeof(State),
    //                       (GLvoid *)(sizeof(glm::vec3) + sizeof(glm::vec4)));
    // glEnableVertexAttribArray(flags);
    // glVertexAttribDivisor(flags, 1);

    glBindVertexArray(0);
  }

  void WriteVertices(const GLfloat *vertices, size_t size) {
    glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
    glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);
  }

  void WriteState(const Frame &frame) {
    glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
    glBufferData(GL_ARRAY_BUFFER, frame.size() * sizeof(State), frame.data(),
                 GL_STATIC_DRAW);
    frame_size = frame.size();
  }

  void Draw(const Frame &frame, const glm::mat4 &v, const glm::mat4 &p) {
    WriteState(frame);
    program.Use();

    glUniformMatrix4fv(view, 1, GL_FALSE, glm::value_ptr(v));
    glUniformMatrix4fv(projection, 1, GL_FALSE, glm::value_ptr(p));

    glUniform3f(object_color, 1.0f, 1.0f, 1.0f);
    glUniform3f(light_color, 1.0f, 1.0f, 1.0f);
    glUniform3f(light_position, 0.f, 25.f, 25.f);

    glUniform1f(alpha, 1.0f);

    glBindVertexArray(VAO);
    glDrawArraysInstanced(GL_TRIANGLES, 0, vertices_size / 6, frame_size);
    glBindVertexArray(0);
  }
};

} // namespace render

class Client {
public:
  Client(Loop &loop, Window &window, render::Model &model)
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
  render::Model &model_;
  Window &window_;
  UDP socket_;
  struct sockaddr_in server_addr_;

  bool debug_enabled_;
  UDP debug_socket_;
  Frame debug_frame_;
  struct sockaddr_in debug_addr_;

  glm::mat4 view_;
};

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  Window &window = Window::Default();
  render::Model model;

  Loop loop;
  Client client(loop, window, model);

  return client.ConnectAndRun(FLAGS_server_addr.c_str(), FLAGS_server_port);
}
