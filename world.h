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

class World {
  friend class GameServer;

public:
  World(glm::vec3 gravity);

  ~World();

  void Add(Cube *cube);

  void Player(Cube *cube);

  void Update(float deltaTime);

  void Reset();

  void Dump(std::vector<State> &state);

  void Dump(std::vector<QState> &state);

private:

  void Katamari(Cube *player);

  btDefaultCollisionConfiguration *collisionConfiguration_;
  btCollisionDispatcher *dispatcher_;
  btBroadphaseInterface *broadphase_;
  btSequentialImpulseConstraintSolver *solver_;
  btDiscreteDynamicsWorld *dynamicsWorld_;

  btCollisionShape *groundShape_;
  btDefaultMotionState *groundMotionState_;
  btRigidBody *groundRigidBody_;

  std::vector<Cube *> cubes_;
  std::vector<Cube *> players_;
};