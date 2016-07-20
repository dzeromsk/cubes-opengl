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

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <btBulletDynamicsCommon.h>

#include "state.h"
#include "cube.h"

#include "world.h"

DEFINE_double(force, 9e3f, "Attraction force");
DEFINE_double(force_distance, 4.5f, "Attraction force distance cap");

World::World(glm::vec3 gravity) {
  collisionConfiguration_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collisionConfiguration_);
  broadphase_ = new btDbvtBroadphase();
  solver_ = new btSequentialImpulseConstraintSolver();
  solver_->setRandSeed(0);
  dynamicsWorld_ = new btDiscreteDynamicsWorld(
      dispatcher_, broadphase_, solver_, collisionConfiguration_);
  dynamicsWorld_->setGravity(btVector3(gravity.x, gravity.y, gravity.z));

  groundShape_ = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
  groundMotionState_ = new btDefaultMotionState(
      btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));

  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(
      0, groundMotionState_, groundShape_, btVector3(0, 0, 0));
  groundRigidBody_ = new btRigidBody(groundRigidBodyCI);

  dynamicsWorld_->addRigidBody(groundRigidBody_);
}

World::~World() {
  delete dynamicsWorld_;

  delete groundShape_;
  delete groundMotionState_;

  delete solver_;
  delete broadphase_;
  delete dispatcher_;
  delete collisionConfiguration_;

  for (auto const cube : cubes_) {
    delete cube;
  }
}

void World::Add(Cube *cube) {
  dynamicsWorld_->addRigidBody(cube->body_);
  cubes_.push_back(cube);
}

void World::Player(Cube *cube) {
  players_.push_back(cube);
  Add(cube);
}

void World::Update(float deltaTime) {
  dynamicsWorld_->stepSimulation(deltaTime, 10);

  for (auto p : players_) {
    Katamari(p);
  }
}

void World::Katamari(Cube *player) {
  int bias = player->body_->wantsSleeping() ? -1 : 1;

  for (auto const c : cubes_) {
    auto cube = c->body_;

    if (cube->wantsSleeping()) {
      continue;
    }

    if (cube == player->body_) {
      continue;
    }

    btVector3 difference = player->body_->getCenterOfMassPosition() -
                           cube->getCenterOfMassPosition();

    btScalar distanceSquared = difference.length2();
    btScalar distance = difference.length();

    if (distance < FLAGS_force_distance) {
      btVector3 direction = difference / distance * bias;
      btScalar magnitude = FLAGS_force / distanceSquared;
      cube->applyCentralForce(direction * magnitude);
    }
  }
}

void World::Reset() {
  for (auto const cube : cubes_) {
    cube->Reset();
  }
  dynamicsWorld_->clearForces();
}

void World::Dump(std::vector<State> &state) {
  for (auto const cube : cubes_) {
    State s;
    cube->Dump(&s);
    state.emplace_back(s);
  }
}

void World::Dump(std::vector<QState> &state) {
  for (auto const cube : cubes_) {
    State s;
    cube->Dump(&s);
    state.emplace_back(s);
  }
}