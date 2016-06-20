// The MIT License (MIT)
//
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

#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>
#include <gflags/gflags.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glog/logging.h>
#include <uv.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "cube.h"
#include "cube_model.h"
#include "cube_shader.h"
#include "shader_builder.h"
#include "world.h"

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#include "third_party/nuklear/nuklear.h"
#include "third_party/nuklear/nuklear_glfw_gl3.h"

#define UNUSED(x) (void)(x)
#define MAX_VERTEX_BUFFER 512 * 1024
#define MAX_ELEMENT_BUFFER 128 * 1024

DEFINE_int32(width, 1280, "Window width");
DEFINE_int32(height, 800, "Windows height");
DEFINE_int32(cubes_count, 15, "Small cube dimension");
DEFINE_int32(player_scale, 4, "Player cube scale");
DEFINE_double(player_mass, 1e3f, "Player cube mass");

template <typename type, size_t size = 10> class CircularBuffer {
public:
  CircularBuffer() : n_(0) { memset(array_, 0, sizeof(array_)); }

  void append(const type &value) { array_[(n_++ % size)] = value; }

  type &operator[](int offset) { return array_[(n_ + offset) % size]; }

private:
  type array_[size];
  size_t n_;
};

// static Cube *cube = nullptr;
static World *world = nullptr;
static GLFWwindow *window = nullptr;

void key_callback(GLFWwindow *window, int key, int scancode, int action,
                  int mode) {
  UNUSED(scancode);
  UNUSED(mode);

  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }

  if (key == GLFW_KEY_Q && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }

  if (key == GLFW_KEY_R && action == GLFW_PRESS) {
    world->Reset();
  }

  if (key == GLFW_KEY_W) {
    cube->Force(glm::vec3(0.f, 0.f, -1.f));
  }

  if (key == GLFW_KEY_S) {
    cube->Force(glm::vec3(0.f, 0.f, 1.f));
  }

  if (key == GLFW_KEY_A) {
    cube->Force(glm::vec3(-1.f, 0.f, 0.f));
  }

  if (key == GLFW_KEY_D) {
    cube->Force(glm::vec3(1.f, 0.f, 0.f));
  }

  if (key == GLFW_KEY_T && action == GLFW_PRESS) {
    static bool black = true;
    if (black) {
      glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
      black = false;
    } else {
      glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
      black = true;
    }
  }

  if (key == GLFW_KEY_F11 && action == GLFW_PRESS) {
    static bool fullscreen = false;
    static int w = 0, h = 0;
    if (fullscreen) {
      glfwSetWindowMonitor(window, nullptr, 0, 0, w, h, GLFW_DONT_CARE);
      fullscreen = false;
    } else {
      GLFWmonitor *monitor = glfwGetPrimaryMonitor();
      const GLFWvidmode *mode = glfwGetVideoMode(monitor);
      glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height,
                           mode->refreshRate);
      fullscreen = true;
      w = FLAGS_width;
      h = FLAGS_height;
    }
  }
}

void window_size_callback(GLFWwindow *window, int width, int height) {
  UNUSED(window);
  FLAGS_width = width;
  FLAGS_height = height;
  glViewport(0, 0, width, height);
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  uv_loop_t *loop;
  CHECK(loop = uv_default_loop());

  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4);

  window =
      glfwCreateWindow(FLAGS_width, FLAGS_height, "Cubes", nullptr, nullptr);
  if (window == nullptr) {
    fprintf(stderr, "Failed to Create OpenGL Context");
    return EXIT_FAILURE;
  }

  glfwMakeContextCurrent(window);
  gladLoadGL();
  glfwSetKeyCallback(window, key_callback);
  glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSwapInterval(1);

  GLuint shaderProgram =
      ShaderBuilder().Vertex(vertexSource).Fragment(fragmentSource).Build();

  CubeModel cm(shaderProgram);
  cm.Data(kVertices, kVerticesSize);

  gDeactivationTime = btScalar(1.);

  world = new World(glm::vec3(0, -20, 0));
  for (int i = -FLAGS_cubes_count; i < FLAGS_cubes_count; ++i) {
    for (int j = -FLAGS_cubes_count; j < FLAGS_cubes_count; ++j) {
      glm::vec3 pos = glm::vec3(i * 2, 0.5f, j * 2);
      Cube *c = new Cube(pos, &cm);
      world->Add(c);
    }
  }

  cube =
      new Cube(glm::vec3(0, 15, 0), &cm, FLAGS_player_scale, FLAGS_player_mass);
  world->Add(cube);

  GLfloat deltaTime = 0.0f; // Time between current frame and last frame
  GLfloat lastFrame = 0.0f; // Time of last frame

  glm::mat4 view;
  glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

  int frames = 0;
  GLfloat lastUpdate = 0; // Time of last frame

  struct nk_context *ctx = nk_glfw3_init(window, NK_GLFW3_INSTALL_CALLBACKS);
  struct nk_font_atlas *atlas;
  nk_glfw3_font_stash_begin(&atlas);
  nk_glfw3_font_stash_end();

  glViewport(0, 0, FLAGS_width, FLAGS_height);
  glClearColor(0.1f, 0.1f, 0.1f, 0.0f);

  static char buffer[256] = {0};
  CircularBuffer<int, 30> fps_history;

  while (glfwWindowShouldClose(window) == false) {
    uv_run(loop, UV_RUN_NOWAIT);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    GLfloat currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    if (currentFrame - lastUpdate >= 1.0) {
      snprintf(buffer, sizeof(buffer), "%d", frames);
      fps_history.append(frames);
      frames = 0;
      lastUpdate = currentFrame;
    }
    frames++;

    glfwPollEvents();
    nk_glfw3_new_frame();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::mat4 projection = glm::perspective(
        45.0f, (GLfloat)FLAGS_width / (GLfloat)FLAGS_height, 0.1f, 100.0f);

    auto c = cube->Position();
    glm::mat4 view = glm::lookAt(c + glm::vec3(0.f, 15.f, 25.f), c, cameraUp);

    world->Update(deltaTime);
    world->Draw(view, projection);

    {
      struct nk_panel layout;
      if (nk_begin(ctx, &layout, "Cubes", nk_rect(50, 50, 230, 230),
                   NK_WINDOW_BORDER | NK_WINDOW_MOVABLE |
                       NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE)) {

        nk_layout_row_dynamic(ctx, 50, 1);
        if (nk_chart_begin(ctx, NK_CHART_COLUMN, 15, 0, 1.5f)) {
          for (int i = -15; i < 0; ++i) {
            nk_chart_push(ctx, (float)fps_history[i] / 60);
          }
          nk_chart_end(ctx);
        }

        static const float ratio[] = {60, 135};

        nk_layout_row(ctx, NK_STATIC, 25, 2, ratio);
        nk_label(ctx, "FPS:", NK_TEXT_LEFT);
        int len = strlen(buffer);

        nk_edit_string(ctx, NK_EDIT_SIMPLE | NK_EDIT_READ_ONLY, buffer, &len,
                       128, nk_filter_default);
      }
      nk_end(ctx);
    }

    nk_glfw3_render(NK_ANTI_ALIASING_ON, MAX_VERTEX_BUFFER, MAX_ELEMENT_BUFFER);

    glfwSwapBuffers(window);
  }

  glfwTerminate();
  uv_loop_close(loop);

  return EXIT_SUCCESS;
}
