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

#pragma once

#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>
#include <glm/glm.hpp>

class ShaderBuilder {
public:
  ShaderBuilder() : vertex_shader_(0), fragment_shader_(0) {}

  ShaderBuilder &Vertex(const char *source) {
    GLuint shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    vertex_shader_ = shader;
    return *this;
  }

  ShaderBuilder &Fragment(const char *source) {
    GLuint shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    fragment_shader_ = shader;
    return *this;
  }

  GLuint Build() {
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader_);
    glAttachShader(program, fragment_shader_);
    glLinkProgram(program);

    glDeleteShader(vertex_shader_);
    vertex_shader_ = 0;

    glDeleteShader(fragment_shader_);
    fragment_shader_ = 0;

    return program;
  }

public:
  GLuint vertex_shader_;
  GLuint fragment_shader_;
};
