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
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <functional>
#include <string>
#include <vector>

#pragma pack(push, 1)
struct State {
  glm::vec3 position;
  glm::quat orientation;
  uint32_t interacting;
};
#pragma pack(pop)

typedef std::vector<State> Frame;
typedef std::vector<Frame> Frames;

DEFINE_int32(cubes_count, 901, "Cubes count per frame");
DEFINE_string(logfile, "models.log", "Path to log file");

DEFINE_int32(width, 1280, "Window width");
DEFINE_int32(height, 800, "Windows height");

static Frames &Log() {
  static Frames log;
  return log;
}

static void ReadLog(std::string filename) {
  FILE *f = nullptr;
  CHECK(f = fopen(filename.c_str(), "rb"));
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

class Window {
public:
  static Window &Default() {
    static Window window("TODO", FLAGS_width, FLAGS_height);
    return window;
  }
  ~Window() { glfwTerminate(); }

  void OnResize(std::function<void(int, int)> resize_callback) {
    resize_callback_ = std::move(resize_callback);
  }

  void OnKey(std::function<void(int, int)> key_callback) {
    key_callback_ = std::move(key_callback);
  }

  void Swap() { glfwSwapBuffers(window_); }
  void Poll() { glfwPollEvents(); }

  bool ShouldClose(bool close = false) {
    if (close) {
      glfwSetWindowShouldClose(window_, GL_TRUE);
    }
    return glfwWindowShouldClose(window_);
  }

private:
  Window(const char *title, int width, int height)
      : window_(nullptr), width_(width), height_(height) {
    CHECK(glfwInit() == GLFW_TRUE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    // glfwSwapInterval(1);

    CHECK(window_ = glfwCreateWindow(width_, height_, title, nullptr, nullptr))
        << "Failed to Create OpenGL Context";
    glfwMakeContextCurrent(window_);
    glfwSetWindowUserPointer(window_, this);

    CHECK(gladLoadGL() == GL_TRUE);
    glViewport(0, 0, width_, height_);
    glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    resize_callback_ = [](int a, int b) { printf("Not implemented!\n"); };
    key_callback_ = [](int a, int b) { printf("Not implemented!\n"); };

    glfwSetWindowSizeCallback(window_, Window::ResizeWrapper);
    glfwSetKeyCallback(window_, Window::KeyWrapper);
  };

  static void ResizeWrapper(GLFWwindow *window, int width, int height) {
    ((Window *)glfwGetWindowUserPointer(window))
        ->resize_callback_(width, height);
  }

  static void KeyWrapper(GLFWwindow *window, int key, int scancode, int action,
                         int mode) {
    ((Window *)glfwGetWindowUserPointer(window))->key_callback_(key, action);
  }

  GLFWwindow *window_;
  int width_;
  int height_;
  std::function<void(int, int)> resize_callback_;
  std::function<void(int, int)> key_callback_;
};

namespace render {

#define GLSL(src) "#version 330 core\n" #src

// clang-format off
const char *kVertexSource = GLSL(
  in vec3 position;
  in vec3 normal;
  in mat4 model; // instance

  out vec3 FragPos;
  out vec3 Normal;

  uniform mat4 view;
  uniform mat4 projection;

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

    m[3][3] = 0;
    m[3][3] = 0;
    m[3][3] = 0;
    m[3][3] = 1;
  }

  void main() {
    gl_Position = projection * view * model * vec4(position, 1.0f);
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

  GLint position;
  GLint normal;
  GLint model;

  GLuint VBO[2];
  GLuint VAO;

  size_t vertices_size;
  size_t models_size;

  Model()
      : program(kVertexSource, kFragmentSource), vertices_size(kVerticesSize),
        models_size(0) {
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
#undef GET
#define GET(name)                                                              \
  name = glGetAttribLocation(program.program, #name);                          \
  CHECK(name > -1) << #name;
    GET(position)
    GET(normal)
    GET(model);
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

    // Model attribute
    for (int i = 0; i < 4; i++) {
      glVertexAttribPointer(model + i, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4),
                            (void *)(sizeof(glm::vec4) * i));
      glEnableVertexAttribArray(model + i);
      glVertexAttribDivisor(model + i, 1);
    }

    glBindVertexArray(0);
  }

  void WriteVertices(const GLfloat *vertices, size_t size) {
    glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
    glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);
  }

  void WriteModel(const std::vector<glm::mat4> &models) {
    glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
    glBufferData(GL_ARRAY_BUFFER, models.size() * sizeof(glm::mat4),
                 models.data(), GL_STATIC_DRAW);
    models_size = models.size();
  }

  void Draw(const std::vector<glm::mat4> &models, const glm::mat4 &v,
            const glm::mat4 &p) {
    WriteModel(models);
    program.Use();

    glUniformMatrix4fv(view, 1, GL_FALSE, glm::value_ptr(v));
    glUniformMatrix4fv(projection, 1, GL_FALSE, glm::value_ptr(p));

    glUniform3f(object_color, 1.0f, 1.0f, 1.0f);
    glUniform3f(light_color, 1.0f, 1.0f, 1.0f);
    glUniform3f(light_position, 0.f, 25.f, 25.f);

    glBindVertexArray(VAO);
    glDrawArraysInstanced(GL_TRIANGLES, 0, vertices_size / 6, models_size);
    glBindVertexArray(0);
  }
};

} // namespace render

class Client {
public:
  Client(Loop &loop, Window &window, render::Model &model)
      : loop_(loop), window_(window), model_(model) {
    projection_ = glm::perspective(
        45.0f, (GLfloat)FLAGS_width / (GLfloat)FLAGS_height, 1.0f, 100.0f);
    view_ = glm::lookAt(glm::vec3(0.f, 50.f, 40.f), glm::vec3(0, 0, 0),
                        glm::vec3(0.0f, 1.0f, 0.0f));
  }

  int Run() {
    window_.OnResize([](int w, int h) { glViewport(0, 0, w, h); });
    window_.OnKey([&](int key, int action) {
      if (action == GLFW_PRESS) {
        OnKey(key);
      }
    });

    // TODO(dzeromsk): Send input to server at 30fps
    Timer input(&loop_, [&](Timer *t) { window_.Poll(); }, 32);
    Timer render(&loop_, [&](Timer *t) { OnFrame(); }, 16);

    return loop_.Run();
  }

  void OnKey(int key) {
    switch (key) {
    case GLFW_KEY_ESCAPE:
    case GLFW_KEY_Q:
      loop_.Stop();
      break;
    case GLFW_KEY_1:
      view_ = glm::lookAt(glm::vec3(0.f, 15.f, 25.f), glm::vec3(0, 0, 0),
                          glm::vec3(0.0f, 1.0f, 0.0f));
      break;
    case GLFW_KEY_2:
      view_ = glm::lookAt(glm::vec3(0.f, 50.f, 40.f), glm::vec3(0, 0, 0),
                          glm::vec3(0.0f, 1.0f, 0.0f));
      break;
    default:
      break;
    }
  }

  void OnFrame() {
    Frame *frame = Next();
    if (frame == nullptr) {
      return;
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, FLAGS_width, FLAGS_height);
    glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glBlendFunc(GL_ONE, GL_SRC_ALPHA);
    glDisable(GL_BLEND);

    std::vector<glm::mat4> models;
    for (const auto &cube : *frame) {
      // TODO(dzeromsk): compute model matrix and set color in
      // shader
      glm::quat rot(cube.orientation[3], cube.orientation[0],
                    cube.orientation[1], cube.orientation[2]);

      glm::mat4 translate = glm::translate(glm::mat4(1.0), cube.position);
      glm::mat4 rotate = glm::mat4_cast(rot);

      models.push_back(translate * rotate);
    }

    model_.Draw(models, view_, projection_);

    window_.Swap();
  }

  Frame *Next() {
    static size_t n = 0;
    if (n >= Log().size()) {
      loop_.Stop();
      return nullptr;
    }
    n++;
    return &Log()[n];
  }

private:
  Loop &loop_;
  render::Model &model_;
  Window &window_;
  glm::mat4 view_;
  glm::mat4 projection_;
};

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // TODO(dzeromsk): Replace log with udp packets from server. Server will send
  // state in loop. Then add interpolation and reduce state dumps to 10pps.
  ReadLog(FLAGS_logfile);
  CHECK(Log().size() > 0);

  Window &window = Window::Default();
  render::Model model;

  Loop loop;
  Client client(loop, window, model);

  return client.Run();
}