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

#include "cube.h"

static Cube *cube;

class World {
public:
  World(glm::vec3 gravity) {
    collisionConfiguration_ = new btDefaultCollisionConfiguration();
    dispatcher_ = new btCollisionDispatcher(collisionConfiguration_);
    broadphase_ = new btDbvtBroadphase();
    solver_ = new btSequentialImpulseConstraintSolver();
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

  ~World() {
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

  void Add(Cube *cube) {
    dynamicsWorld_->addRigidBody(cube->GetBody());
    cubes_.push_back(cube);
  }

  void Update(GLfloat deltaTime) {
    dynamicsWorld_->stepSimulation(deltaTime, 10);

    // TODO(dzeromks): Cleanup, move some code up to Cube object
    auto bigCube = cube->GetBody();

    int bias = 1;
    if (bigCube->wantsSleeping()) {
      bias = -1;
    }

    for (auto const c : cubes_) {
      auto cube = c->GetBody();
      if (cube->wantsSleeping()) {
        continue;
      }

      if (cube == bigCube) {
        continue;
      }

      btVector3 difference =
          bigCube->getCenterOfMassPosition() - cube->getCenterOfMassPosition();

      btScalar distanceSquared = difference.length2();
      btScalar distance = difference.length();

      if (distance < 4.0f) {
        btVector3 direction = difference / distance * bias;
        btScalar magnitude = 10e3f / distanceSquared;
        cube->applyCentralForce(direction * magnitude);
      }
    }
  }

  void Draw(const glm::mat4 &view, const glm::mat4 &projection) {
    for (auto const cube : cubes_) {
      cube->Draw(view, projection);
    }
  }

  void Draw(const int model, const glm::mat4 &view,
            const glm::mat4 &projection) {
    for (auto const cube : cubes_) {
      cube->Draw(model, view, projection);
    }
  }

  void Reset() {
    for (auto const cube : cubes_) {
      cube->Reset();
    }
    dynamicsWorld_->clearForces();
  }

private:
  btDefaultCollisionConfiguration *collisionConfiguration_;
  btCollisionDispatcher *dispatcher_;
  btBroadphaseInterface *broadphase_;
  btSequentialImpulseConstraintSolver *solver_;
  btDiscreteDynamicsWorld *dynamicsWorld_;

  btCollisionShape *groundShape_;
  btDefaultMotionState *groundMotionState_;
  btRigidBody *groundRigidBody_;

  std::vector<Cube *> cubes_;
};