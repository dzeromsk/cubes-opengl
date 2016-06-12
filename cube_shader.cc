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

#include "cube_shader.h"

#define GLSL(src) "#version 330 core\n" #src

const char *vertexSource = GLSL(

    layout(location = 0) in vec3 position; layout(location = 1) in vec3 normal;

    out vec3 FragPos; out vec3 Normal;

    uniform mat4 model; uniform mat4 view; uniform mat4 projection;

    void main() {
      gl_Position = projection * view * model * vec4(position, 1.0f);
      FragPos = vec3(model * vec4(position, 1.0f));
      Normal = vec3(model * vec4(normal, 0));
    });

const char *fragmentSource = GLSL(

    in vec3 Normal; in vec3 FragPos;

    out vec4 color;

    uniform vec3 lightPos; uniform vec3 lightColor; uniform vec3 objectColor;

    void main() {
      // Ambient
      float ambientStrength = 0.1f;
      vec3 ambient = ambientStrength * lightColor;

      // Diffuse
      vec3 norm = normalize(Normal);
      vec3 lightDir = normalize(lightPos - FragPos);
      float diff = max(dot(norm, lightDir), 0.0);
      // float diff = max(dot(norm, vec3(0.0, 1.0, 0.0)), 0.0);
      vec3 diffuse = diff * lightColor;

      vec3 result = (ambient + diffuse) * objectColor;
      color = vec4(result, 1.0f);
      // color = vec4(norm*.5f + .5f, 1.0f);
    });
