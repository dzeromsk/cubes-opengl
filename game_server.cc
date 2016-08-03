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
#include <uv.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <btBulletDynamicsCommon.h>

#include <functional>
#include <map>
#include <set>

#include "base/loop.h"
#include "base/idle.h"
#include "base/timer.h"
#include "net/addr.h"
#include "net/udp.h"
#include "net/udp_server.h"
#include "state.h"
#include "cube.h"
#include "world.h"

#include "game_server.h"

DEFINE_int32(cubes_count, 15, "Small cube dimension");
DEFINE_double(player_mass, 1e3f, "Player cube mass");

GameServer::GameServer()
    : server_(loop_), debug_server_(loop_), timer_(loop_), seq_(0),
      world_(glm::vec3(0, -20, 0)) {
  server_.OnReceive(
      [&](const uv_buf_t buf, const Addr addr) { OnReceive(buf, addr); });

  debug_server_.OnReceive(
      [&](const uv_buf_t buf, const Addr addr) { OnDebugReceive(buf, addr); });

  for (int i = -FLAGS_cubes_count; i < FLAGS_cubes_count; ++i) {
    for (int j = -FLAGS_cubes_count; j < FLAGS_cubes_count; ++j) {
      world_.Add(new Cube(glm::vec3(i * 2, 0.5f, j * 2)));
    }
  }

  world_.Reset();
}

int GameServer::ListenAndServe(const char *ip, int port) {
  timer_.Start(
      [&] {
        world_.Update(16.0f / 1000);
        frame_.clear();
        world_.Dump(frame_);

        qframe_.clear();
        world_.Dump(qframe_);

        OnTick();
        OnDebugTick();
      },
      16);

  debug_server_.Listen(ip, port + 1);
  // we use the same loop so as an side effect ListenAndServe will start debug
  // server as well
  return server_.ListenAndServe(ip, port);
}

Frame &GameServer::Next(int n) {
  // return Log()[n % Log().size()];
  return frame_;
}

void GameServer::ApplyForce(const uv_buf_t &request, const Addr &addr) {
  switch (request.base[0]) {
  case 'w':
    players_[addr]->Force(glm::vec3(0.f, 0.f, -1.f));
    break;
  case 's':
    players_[addr]->Force(glm::vec3(0.f, 0.f, 1.f));
    break;
  case 'a':
    players_[addr]->Force(glm::vec3(-1.f, 0.f, 0.f));
    break;
  case 'd':
    players_[addr]->Force(glm::vec3(1.f, 0.f, 0.f));
    break;
  case 'r':
    world_.Reset();
    break;
  default:
    break;
  }
}

void GameServer::OnReceive(uv_buf_t request, Addr addr) {
  auto status = clients_.emplace(addr);
  if (status.second) {
    // printf("New client\n");
    NewPlayer(addr);
  }

  ApplyForce(request, addr);
}

void GameServer::NewPlayer(Addr addr) {
  auto player = new Cube(glm::vec3(0, 15, 0), 1, FLAGS_player_mass);
  // world takes ownership of the player cube
  world_.Player(player);
  players_[addr] = player;
}

void GameServer::OnTick() {
  Next(seq_);
  seq_++;

  if (!(seq_ % 6)) {
    size_t state_size = qframe_.size() * sizeof(QState);
    size_t total_size = sizeof(Packet) + state_size;

    Packet *p = (Packet *)Alloc(total_size);
    p->seq = seq_;
    p->size = qframe_.size();
    memcpy(p->data, qframe_.data(), state_size);

    uv_buf_t response = { 0 };
	response.base = (char *)p;
	response.len = total_size;

    for (const auto &client : clients_) {
      server_.Send(client, &response);
    }
  }
}

void GameServer::OnDebugReceive(uv_buf_t request, Addr addr) {
  debug_clients_.emplace(addr);
}

void GameServer::OnDebugTick() {
  Frame &frame = Next(seq_);

  uv_buf_t response = { 0 };
  response.base = (char*)frame.data();
  response.len = frame.size() * sizeof(State);
  for (const auto &client : debug_clients_) {
    debug_server_.Send(client, &response);
  }
}

char *GameServer::Alloc(int size) {
  static char slab[65536];
  CHECK(size < 65536);
  return slab;
}