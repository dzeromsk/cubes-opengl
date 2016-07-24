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

#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include <functional>

#include "window.h"

DEFINE_int32(width, 1280, "Window width");
DEFINE_int32(height, 800, "Windows height");

Window &Window::Default() {
  static Window window("TODO", FLAGS_width, FLAGS_height);
  return window;
}

Window::~Window() { glfwTerminate(); }

void Window::OnResize(std::function<void(int, int)> resize_callback) {
  resize_callback_ = std::move(resize_callback);
}

void Window::OnKey(std::function<void(int, int)> key_callback) {
  key_callback_ = std::move(key_callback);
}

void Window::Swap() { glfwSwapBuffers(window_); }
void Window::Poll() { glfwPollEvents(); }

bool Window::ShouldClose(bool close) {
  if (close) {
    glfwSetWindowShouldClose(window_, GL_TRUE);
  }
  return glfwWindowShouldClose(window_);
}

Window::Window(const char *title, int width, int height)
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

  resize_callback_ = [](int a, int b) {};
  key_callback_ = [](int a, int b) {};

  glfwSetWindowSizeCallback(window_, Window::ResizeWrapper);
  glfwSetKeyCallback(window_, Window::KeyWrapper);
};

void Window::ResizeWrapper(GLFWwindow *window, int width, int height) {
  ((Window *)glfwGetWindowUserPointer(window))->resize_callback_(width, height);
}

void Window::KeyWrapper(GLFWwindow *window, int key, int scancode, int action,
                        int mode) {
  ((Window *)glfwGetWindowUserPointer(window))->key_callback_(key, action);
}
