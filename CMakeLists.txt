# Copyright (c) 2016 Dominik Zeromski
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required (VERSION 3.0)
project (cubes)

option (GLFW_BUILD_DOCS OFF)
option (GLFW_BUILD_EXAMPLES OFF)
option (GLFW_BUILD_TESTS OFF)
add_subdirectory (third_party/glfw)

option (BUILD_BULLET2_DEMOS OFF)
option (BUILD_CPU_DEMOS OFF)
option (BUILD_EXTRAS OFF)
option (BUILD_OPENGL3_DEMOS OFF)
option (BUILD_UNIT_TESTS OFF)
add_subdirectory (third_party/bullet)

option (LIBUV_BUILD_TESTS OFF)
add_subdirectory (third_party/libuv)

option (BUILD_SHARED_LIBS OFF)
option (BUILD_gflags_nothreads_LIB OFF)
add_subdirectory (third_party/gflags)

option (WITH_GFLAGS ON)
option (WITH_GFLAGS WITH_THREADS)
set (gflags_FOUND 1)
set (gflags_LIBRARIES gflags)
set (HAVE_LIB_GFLAGS 1)
add_subdirectory (third_party/glog)

add_library (glad
  third_party/glad/src/glad.c
)

target_include_directories (glad PUBLIC
  third_party/glad/include/
)

add_executable (client
  base/loop.h
  base/timer.h
  base/window.h
  net/udp.h
  state.h
  shader.h
  program.h
  model.h
  game_client.h
  client.cc
  model.cc
  game_client.cc
  base/window.cc
)

target_include_directories (client PUBLIC
  third_party/libuv/libuv/include/
  third_party/glad/include/
  third_party/glfw/include/
  third_party/gflags/include/
  third_party/glog/
  third_party/glm/
)

target_compile_definitions (client PUBLIC
  -DGLFW_INCLUDE_NONE
)

target_link_libraries (client
  uv
  glfw
  glad
  glog
  gflags
  ${GLFW_LIBRARIES} 
)

if (WIN32)
  target_compile_options (client PUBLIC /W4 /fp:strict)
  target_link_libraries (client ws2_32 iphlpapi psapi userenv)
endif (WIN32)

if (UNIX)
  target_compile_options (client PUBLIC -g -Wextra -std=c++14)
  target_link_libraries (client dl)
endif (UNIX)

add_executable (server
  base/loop.h
  net/udp.h
  net/addr.h
  net/udp_server.h
  base/timer.h
  state.h
  cube.h
  world.h
  game_server.h
  server.cc
  game_server.cc
  world.cc
  cube.cc
)

target_include_directories (server PUBLIC
  third_party/libuv/libuv/include/
  third_party/gflags/include/
  third_party/glog/
  third_party/glm/
  third_party/bullet/src/
)

target_link_libraries (server
  uv
  glog
  gflags
  BulletDynamics
  BulletCollision
  LinearMath
)

if (WIN32)
  target_compile_options (server PUBLIC /W4 /fp:strict)
  target_link_libraries (server ws2_32 iphlpapi psapi userenv)
endif (WIN32)

if (UNIX)
  target_compile_options (server PUBLIC -g -Wextra -std=c++14)
endif (UNIX)

add_executable (demo
  base/loop.h
  base/timer.h
  base/window.cc
  base/window.h
  cube.cc
  cube.h
  demo.cc
  game_client.cc
  game_client.h
  game_server.cc
  game_server.h
  model.cc
  model.h
  net/addr.h
  net/udp.h
  net/udp_server.h
  program.h
  shader.h
  state.h
  world.cc
  world.h
  menu.h
  menu.cc
)

target_compile_definitions (demo PUBLIC
  -DGLFW_INCLUDE_NONE
)

target_include_directories (demo PUBLIC
  third_party/bullet/src/
  third_party/gflags/include/
  third_party/glad/include/
  third_party/glfw/include/
  third_party/glm/
  third_party/glog/
  third_party/libuv/libuv/include/
)

target_link_libraries (demo
  BulletDynamics
  BulletCollision
  LinearMath
  gflags
  glad
  glfw
  ${GLFW_LIBRARIES}
  glog
  uv
)

if (WIN32)
  target_compile_options (demo PUBLIC /W4 /fp:strict)
  target_link_libraries (demo ws2_32 iphlpapi psapi userenv)
endif (WIN32)

if (UNIX)
  target_compile_options (demo PUBLIC -g -Wextra -std=c++14)
endif (UNIX)

add_executable (udp_test
  udp_test.cc
)

target_include_directories (udp_test PUBLIC
  third_party/gflags/include/
  third_party/glog/
  third_party/libuv/libuv/include/
)

target_link_libraries (udp_test
  gflags
  glog
  uv
)

if (WIN32)
  target_compile_options (udp_test PUBLIC /W4 /fp:strict)
  target_link_libraries (udp_test ws2_32 iphlpapi psapi userenv)
endif (WIN32)

if (UNIX)
  target_compile_options (udp_test PUBLIC -g -Wextra -std=c++14)
endif (UNIX)