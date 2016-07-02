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
#include "hud.h"
#include "shader_builder.h"
#include "world.h"

#define UNUSED(x) (void)(x)

DEFINE_int32(width, 1280, "Window width");
DEFINE_int32(height, 800, "Windows height");
DEFINE_int32(cubes_count, 15, "Small cube dimension");
DEFINE_int32(player_scale, 4, "Player cube scale");
DEFINE_double(player_mass, 1e3f, "Player cube mass");

static GLFWwindow *window = nullptr;

#define INPUT_UP (1 << 0)
#define INPUT_DOWN (1 << 1)
#define INPUT_LEFT (1 << 3)
#define INPUT_RIGHT (1 << 2)
#define INPUT_RESET (1 << 4)
#define INPUT_QUIT (1 << 5)
static uint8_t input;
static CircularBuffer<uint8_t, 60> input_history;

void handle_input(World *world, Cube *cube, uint8_t input) {
  if (input & INPUT_UP) {
    cube->Force(glm::vec3(0.f, 0.f, -1.f));
  }
  if (input & INPUT_DOWN) {
    cube->Force(glm::vec3(0.f, 0.f, 1.f));
  }
  if (input & INPUT_LEFT) {
    cube->Force(glm::vec3(-1.f, 0.f, 0.f));
  }
  if (input & INPUT_RIGHT) {
    cube->Force(glm::vec3(1.f, 0.f, 0.f));
  }
  if (input & INPUT_RESET) {
    world->Reset();
  }
  // if (input & INPUT_QUIT) {
  // }
}

void key_callback(GLFWwindow *window, int key, int scancode, int action,
                  int mode) {
  UNUSED(scancode);
  UNUSED(mode);

  input = 0;

  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
    input |= INPUT_QUIT;
  }

  if (key == GLFW_KEY_Q && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
    input |= INPUT_QUIT;
  }

  if (key == GLFW_KEY_R && action == GLFW_PRESS) {
    input |= INPUT_RESET;
  }

  if (key == GLFW_KEY_W) {
    input |= INPUT_UP;
  }

  if (key == GLFW_KEY_S) {
    input |= INPUT_DOWN;
  }

  if (key == GLFW_KEY_A) {
    input |= INPUT_LEFT;
  }

  if (key == GLFW_KEY_D) {
    input |= INPUT_RIGHT;
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

  CHECK(glfwInit() == GLFW_TRUE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4);

  CHECK(window = glfwCreateWindow(FLAGS_width, FLAGS_height, "Cubes", nullptr,
                                  nullptr))
      << "Failed to Create OpenGL Context";

  glfwMakeContextCurrent(window);

  CHECK(gladLoadGL() == GL_TRUE);

  glfwSetKeyCallback(window, key_callback);
  glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSwapInterval(1);

  GLuint shaderProgram =
      ShaderBuilder().Vertex(vertexSource).Fragment(fragmentSource).Build();

  CubeModel cm(shaderProgram);
  cm.Data(kVertices, kVerticesSize);

  gDeactivationTime = btScalar(1.);

  auto world = new World(glm::vec3(0, -20, 0));
  for (int i = -FLAGS_cubes_count; i < FLAGS_cubes_count; ++i) {
    for (int j = -FLAGS_cubes_count; j < FLAGS_cubes_count; ++j) {
      glm::vec3 pos = glm::vec3(i * 2, 0.5f, j * 2);
      Cube *c = new Cube(pos, &cm);
      world->Add(c);
    }
  }

  auto cube =
      new Cube(glm::vec3(0, 15, 0), &cm, FLAGS_player_scale, FLAGS_player_mass);
  world->Player(cube);

  GLfloat deltaTime = 0.0f; // Time between current frame and last frame
  GLfloat lastFrame = 0.0f; // Time of last frame

  glm::mat4 view;
  glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

  auto hud = new HUD(window);

  glViewport(0, 0, FLAGS_width, FLAGS_height);
  glClearColor(0.1f, 0.1f, 0.1f, 0.0f);

  while (glfwWindowShouldClose(window) == false) {
    uv_run(loop, UV_RUN_NOWAIT);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glBlendFunc(GL_ONE, GL_SRC_ALPHA);

    GLfloat currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    input = 0;
    glfwPollEvents();
    handle_input(world, cube, input_history[hud->GetInputDelay() * -1]);
    input_history.append(input);

    // glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::mat4 projection = glm::perspective(
        45.0f, (GLfloat)FLAGS_width / (GLfloat)FLAGS_height, 0.1f, 100.0f);

    auto c = cube->Position();
    glm::mat4 view = glm::lookAt(c + glm::vec3(0.f, 15.f, 25.f), c, cameraUp);

    world->Update(deltaTime);
    world->Draw(view, projection);

    // glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    world->Draw(hud->GetDelay() * -1, view, projection);

    hud->Draw();

    glfwSwapBuffers(window);
  }

  glfwTerminate();
  uv_loop_close(loop);

  return EXIT_SUCCESS;
}
