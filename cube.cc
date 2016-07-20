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

#include <btBulletDynamicsCommon.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "state.h"

#include "cube.h"

DEFINE_int32(max_speed, 18, "Max speed cap");
DEFINE_double(default_mass, 10, "Default cube mass");

Cube::Cube(glm::vec3 position, float scale, float mass) : scale_(scale) {
  shape_ = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f) * scale_);
  motionState_ = new btDefaultMotionState(btTransform(
      btQuaternion(0, 0, 0, 1), btVector3(position.x, position.y, position.z)));

  initial_position_ = position;

  // btScalar mass = 10 * scale_;
  btVector3 inertia(0, 0, 0);
  shape_->calculateLocalInertia(mass, inertia);

  btRigidBody::btRigidBodyConstructionInfo bodyCI(mass, motionState_, shape_,
                                                  inertia);
  bodyCI.m_friction = 100;
  bodyCI.m_restitution = .0f;
  body_ = new btRigidBody(bodyCI);

  // TODO(dzeromsk): find better api to start object inactive
  if (scale_ == 1) {
    body_->updateDeactivation(10.0f);
  }
}

Cube::~Cube() {
  delete body_;
  delete motionState_;
  delete shape_;
}

void Cube::Reset() {
  btTransform transform = body_->getCenterOfMassTransform();
  transform.setOrigin(
      btVector3(initial_position_.x, initial_position_.y, initial_position_.z));
  transform.setRotation(btQuaternion(0, 0, 0, 1));
  body_->setCenterOfMassTransform(transform);
  body_->getMotionState()->setWorldTransform(transform);
  body_->setLinearVelocity(btVector3(0, 0, 0));
  body_->setAngularVelocity(btVector3(0, 0, 0));
  body_->clearForces();
  body_->activate(true);
  if (scale_ == 1) {
    body_->updateDeactivation(10.0f);
  }
}

void Cube::Force(const glm::vec3 &i) {
  body_->activate(true);
  auto a = body_->getLinearVelocity();
  if (a.length() < FLAGS_max_speed) {
    body_->setLinearVelocity(a + btVector3(i.x, i.y, i.z));
  }
}

void Cube::Dump(State *state) {
  const btVector3 &position = body_->getCenterOfMassPosition();
  state->position[0] = position.getX();
  state->position[1] = position.getY();
  state->position[2] = position.getZ();

  const btQuaternion orientation = body_->getOrientation();
  state->orientation[0] = orientation.getX();
  state->orientation[1] = orientation.getY();
  state->orientation[2] = orientation.getZ();
  state->orientation[3] = orientation.getW();

  state->interacting = !body_->wantsSleeping();
}