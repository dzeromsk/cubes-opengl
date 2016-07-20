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

class Window {
public:
  static Window &Default();

  ~Window();

  void OnResize(std::function<void(int, int)> resize_callback);

  void OnKey(std::function<void(int, int)> key_callback);

  void Swap();
  
  void Poll();

  bool ShouldClose(bool close = false);

private:
  Window(const char *title, int width, int height);

  static void ResizeWrapper(GLFWwindow *window, int width, int height);

  static void KeyWrapper(GLFWwindow *window, int key, int scancode, int action,
                         int mode);

  GLFWwindow *window_;
  int width_;
  int height_;
  std::function<void(int, int)> resize_callback_;
  std::function<void(int, int)> key_callback_;
};