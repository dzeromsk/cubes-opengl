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
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

extern const char *vertexSource;
extern const char *fragmentSource;

class CubeShader {
public:
  CubeShader(GLuint program) {
    glUseProgram(program);

    object_color_location_ = glGetUniformLocation(program, "objectColor");
    light_color_location_ = glGetUniformLocation(program, "lightColor");
    light_position_location_ = glGetUniformLocation(program, "lightPos");
    model_location_ = glGetUniformLocation(program, "model");
    view_location_ = glGetUniformLocation(program, "view");
    projection_location_ = glGetUniformLocation(program, "projection");
  }

  void ObjectColor(const glm::vec3 &color) {
    glUniform3f(object_color_location_, color.x, color.y, color.z);
  }

  void LightColor(const glm::vec3 &color) {
    glUniform3f(light_color_location_, color.x, color.y, color.z);
  }

  void LightPosition(const glm::vec3 &position) {
    glUniform3f(light_position_location_, position.x, position.y, position.z);
  }

  void Model(const glm::mat4 &model) {
    glUniformMatrix4fv(model_location_, 1, GL_FALSE, glm::value_ptr(model));
  }

  void View(const glm::mat4 &view) {
    glUniformMatrix4fv(view_location_, 1, GL_FALSE, glm::value_ptr(view));
  }

  void Projection(const glm::mat4 &projection) {
    glUniformMatrix4fv(projection_location_, 1, GL_FALSE,
                       glm::value_ptr(projection));
  }

private:
  // fragment args
  GLuint object_color_location_;
  GLuint light_color_location_;
  GLuint light_position_location_;

  // vertex args
  GLuint model_location_;
  GLuint view_location_;
  GLuint projection_location_;
};
