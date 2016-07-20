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

#include <glog/logging.h>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>

#include "shader.h"
#include "program.h"
#include "state.h"

#include "model.h"

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

Model::Model()
    : program(kVertexSource, kFragmentSource), vertices_size(kVerticesSize),
      frame_size(0) {
  glGenVertexArrays(1, &VAO);
  glGenBuffers(2, VBO);
  Locations();
  Attrib();
  WriteVertices(kVertices, kVerticesSize);
}

Model::~Model() {
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(2, VBO);
}

void Model::Locations() {
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

void Model::Attrib() {
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

  glVertexAttribPointer(pos, 3, GL_FLOAT, GL_FALSE, sizeof(State), (GLvoid *)0);
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

void Model::WriteVertices(const GLfloat *vertices, size_t size) {
  glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
  glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);
}

void Model::WriteState(const Frame &frame) {
  glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
  glBufferData(GL_ARRAY_BUFFER, frame.size() * sizeof(State), frame.data(),
               GL_STATIC_DRAW);
  frame_size = frame.size();
}

void Model::Draw(const Frame &frame, const glm::mat4 &v, const glm::mat4 &p) {
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