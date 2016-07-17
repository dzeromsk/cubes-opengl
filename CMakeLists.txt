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

add_library (nuklear
  third_party/nuklear/nuklear.cc
)

target_include_directories (nuklear PUBLIC
  third_party/glad/include/
  third_party/glfw/include/
)

add_executable (cubes
  hud.h
  world.h
  cube.h
  cube_model.h
  cube_model.cc
  cube_shader.h
  cube_shader.cc
  shader_builder.h
  cubes.cc
)

target_include_directories (cubes PUBLIC
  third_party/libuv/libuv/include/
  third_party/bullet/src/
  third_party/glad/include/
  third_party/glfw/include/
  third_party/gflags/include/
  third_party/glog/
  third_party/glm/
)

target_compile_definitions (cubes PUBLIC
  -DGLFW_INCLUDE_NONE
)

target_link_libraries (cubes
  uv
  glfw
  glad
  glog
  gflags
  nuklear
  BulletDynamics
  BulletCollision
  LinearMath
  ${GLFW_LIBRARIES} 
)

if (WIN32)
  target_compile_options (cubes PUBLIC /W4 /fp:strict)
  target_link_libraries (cubes ws2_32 iphlpapi psapi userenv)
endif (WIN32)

if (UNIX)
  target_compile_options (cubes PUBLIC -g -Wextra -std=c++14)
  target_link_libraries (cubes dl)
endif (UNIX)

add_executable (client
  client.cc
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
  nuklear
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
  server.cc
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

add_executable (testx
  test.cc
)

target_include_directories (testx PUBLIC
  third_party/libuv/libuv/include/
  third_party/gflags/include/
  third_party/glog/
  third_party/glm/
)

target_link_libraries (testx
  uv
  glog
  gflags
)

if (WIN32)
  target_compile_options (testx PUBLIC /W4 /fp:strict)
  target_link_libraries (testx ws2_32 iphlpapi psapi userenv)
endif (WIN32)

if (UNIX)
  target_compile_options (testx PUBLIC -g -Wextra -std=c++14)
endif (UNIX)