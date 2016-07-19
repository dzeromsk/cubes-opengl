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

#include <btBulletDynamicsCommon.h>
#include <gflags/gflags.h>
#include <glm/glm.hpp>
#include <glog/logging.h>
#include <uv.h>

#include <functional>
#include <map>
#include <set>

#define container_of(ptr, type, member)                                        \
  ((type *)((char *)(ptr)-offsetof(type, member)))

inline uint32_t quantize(float x, int max_bits) {
  return x * ((1 << max_bits) - 1) + 0.5;
}

inline float dequantize(uint32_t x, int max_bits) {
  return float(x) / ((1 << max_bits) - 1);
}

inline float bound(float x, float min, float max) {
  return (x - min) / (max - min);
}

inline float unbound(float x, float min, float max) {
  return x * (max - min) + min;
}

#pragma pack(push, 1)
struct State {
  glm::vec3 position;
  glm::vec4 orientation;
  uint32_t interacting;
};
struct QState {
  int orientation_largest;
  int orientation[3];
  int position[3];
  int interacting;

  QState(const State &s);
};
#pragma pack(pop)

typedef std::vector<State> Frame;
typedef std::vector<Frame> Frames;

typedef std::vector<QState> QFrame;

QState::QState(const State &s) {
  position[0] = quantize(bound(s.position[0], -64, 64), 12);
  position[1] = quantize(bound(s.position[1], -1, 31), 8);
  position[2] = quantize(bound(s.position[2], -64, 64), 12);

  // interacting = !!s.interacting;
  interacting = s.interacting;

  // orientation smallest three method

  // find largest dimension
  float max = 0.0f;
  uint8_t maxno = 0;
  for (int i = 0; i < 4; i++) {
    float v = fabs(s.orientation[i]);
    if (v > max) {
      max = v;
      maxno = i;
    }
  }
  orientation_largest = maxno;

  // save remaining dimensions
  glm::vec3 abc(0.0f);
  for (int i = 0, j = 0; i < 4; i++) {
    if (i != maxno) {
      abc[j++] = s.orientation[i];
    }
  }

  // remeber to handle sign
  if (s.orientation[maxno] < 0) {
    abc *= -1;
  }

  float minimum = -1.0f / 1.414214f; // 1.0f / sqrt(2)
  float maximum = +1.0f / 1.414214f;

  for (int i = 0; i < 3; i++) {
    orientation[i] = quantize(bound(abc[i], minimum, maximum), 7);
  }
}

DEFINE_int32(log_cubes_count, 901, "Cubes count per frame");
DEFINE_string(logfile, "models.log", "Path to log file");

DEFINE_string(server_addr, "127.0.0.1", "Server ip address");
DEFINE_int32(server_port, 3389, "Server port");

DEFINE_double(force, 9e3f, "Attraction force");
DEFINE_double(force_distance, 4.5f, "Attraction force distance cap");

DEFINE_int32(max_speed, 18, "Max speed cap");
DEFINE_double(default_mass, 10, "Default cube mass");
DEFINE_int32(cubes_count, 15, "Small cube dimension");
DEFINE_int32(player_scale, 4, "Player cube scale");
DEFINE_double(player_mass, 1e3f, "Player cube mass");

static Frames &Log() {
  static Frames log;
  return log;
}

static void ReadLog(std::string filename) {
  FILE *f = nullptr;
  CHECK(f = fopen(filename.c_str(), "rb"));
  Log().clear();
  for (int frame = 1; frame; frame++) {
    std::vector<State> cubes;
    for (int i = 0; i < FLAGS_log_cubes_count; i++) {
      State s;
      if (fread(&s, sizeof(State), 1, f) != 1) {
        break;
      }
      cubes.push_back(s);
    }
    if (feof(f)) {
      break;
    }
    Log().push_back(cubes);
  }
  fclose(f);
}

class Loop {
  friend class UDP;
  friend class Timer;

public:
  Loop() { CHECK(uv_loop_init(&loop_) == 0); }
  ~Loop() { uv_loop_close(&loop_); }

  int Run() { return uv_run(&loop_, UV_RUN_DEFAULT); }
  void Stop() { uv_stop(&loop_); }

private:
  uv_loop_t loop_;
};

typedef struct {
  uv_udp_send_t req;
  char *data;
} udp_send_ctx_t;

class UDP {
public:
  UDP(Loop &loop) {
    CHECK(uv_udp_init(&loop.loop_, &socket_) == 0);
    socket_.data = this;
  }

  ~UDP() { uv_udp_recv_stop(&socket_); }

  void Listen(const char *ip, int port) {
    struct sockaddr_in addr;
    CHECK(uv_ip4_addr(ip, port, &addr) == 0);
    CHECK(uv_udp_bind(&socket_, (const struct sockaddr *)&addr, 0) == 0);
    CHECK(uv_udp_recv_start(&socket_, UDP::Alloc, UDP::ReceiveWrapper) == 0);
  }

  typedef std::function<void()> CloseFunc;
  typedef std::function<void(uv_buf_t, const struct sockaddr *, unsigned)>
      ReceiveFunc;

  void OnReceive(ReceiveFunc on_receive) { receive_ = std::move(on_receive); }

  int Send(const uv_buf_t bufs[], unsigned int nbufs,
           const struct sockaddr *addr) {
    udp_send_ctx_t *req = (udp_send_ctx_t *)malloc(sizeof(*req));
    uv_udp_send(&(req->req), &socket_, bufs, nbufs, addr, UDP::Send);
  }

private:
  static void CloseWrapper(uv_handle_t *handle) {
    ((UDP *)handle->data)->close_();
  }

  static void ReceiveWrapper(uv_udp_t *handle, ssize_t nread,
                             const uv_buf_t *buf, const struct sockaddr *addr,
                             unsigned flags) {
    if (nread > 0 && addr != nullptr) {
      uv_buf_t data = {buf->base, size_t(nread)};
      ((UDP *)handle->data)->receive_(data, addr, flags);
    }
  }

  static void Alloc(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf) {
    static char slab[65536];
    *buf = uv_buf_init(slab, sizeof(slab));
  }

  static void Send(uv_udp_send_t *req, int status) {
    udp_send_ctx_t *ctx = container_of(req, udp_send_ctx_t, req);
    free(ctx);
  }

  CloseFunc close_;
  ReceiveFunc receive_;

  uv_udp_t socket_;
};

struct Addr {
  uint32_t ip;
  uint16_t port;

  Addr(const struct sockaddr *addr) {
    ip = ((struct sockaddr_in *)addr)->sin_addr.s_addr;
    port = ntohs(((struct sockaddr_in *)addr)->sin_port);
  }

  bool operator<(const Addr &rhs) const {
    return ip < rhs.ip || (!(rhs.ip < ip) && port < rhs.port);
  }

  bool operator!=(const Addr &rhs) const {
    return ip != rhs.ip || port != rhs.port;
  }

  void Sock(struct sockaddr_in *addr) const {
    memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    addr->sin_addr.s_addr = ip;
  }
};

class UDPServer {
public:
  UDPServer(Loop &loop) : loop_(loop), socket_(loop_) {}

  typedef std::function<void(const uv_buf_t, const Addr)> ReceiveFunc;

  UDPServer(Loop &loop, ReceiveFunc on_receive)
      : loop_(loop), socket_(loop), on_receive_(std::move(on_receive)) {
    socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                          unsigned flags) { on_receive_(buf, addr); });
  }

  void OnReceive(ReceiveFunc on_receive) {
    on_receive_ = on_receive;
    socket_.OnReceive([&](uv_buf_t buf, const struct sockaddr *addr,
                          unsigned flags) { on_receive_(buf, addr); });
  }

  void Listen(const char *ip, int port) { socket_.Listen(ip, port); }

  int ListenAndServe(const char *ip, int port) {
    socket_.Listen(ip, port);
    return loop_.Run();
  }

  int Send(const Addr &client, const uv_buf_t *buf) {
    struct sockaddr_in addr;
    client.Sock(&addr);
    socket_.Send(buf, 1, (struct sockaddr *)&addr);
  }

protected:
  Loop &loop_;
  UDP socket_;
  ReceiveFunc on_receive_;
};

class Timer {
public:
  Timer(Loop &loop) {
    timer_.data = this;
    CHECK(uv_timer_init(&loop.loop_, &timer_) == 0);
  }

  Timer(Loop *loop, std::function<void(void)> callback, uint64_t repeat)
      : callback_(std::move(callback)) {
    timer_.data = this;
    CHECK(uv_timer_init(&loop->loop_, &timer_) == 0);
    CHECK(uv_timer_start(&timer_, Timer::Wrapper, 1, repeat) == 0);
  }

  ~Timer() { uv_timer_stop(&timer_); }

  void Start(std::function<void(void)> callback, uint64_t repeat) {
    callback_ = std::move(callback);
    CHECK(uv_timer_start(&timer_, Timer::Wrapper, 1, repeat) == 0);
  }

private:
  static void Wrapper(uv_timer_t *handle) {
    ((Timer *)handle->data)->callback_();
  }
  std::function<void(void)> callback_;

  uv_timer_t timer_;
};

class Cube {
  friend class World;

public:
  Cube(glm::vec3 position, float scale = 1, float mass = FLAGS_default_mass)
      : scale_(scale) {
    shape_ = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f) * scale_);
    motionState_ = new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1),
                    btVector3(position.x, position.y, position.z)));

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

  ~Cube() {
    delete body_;
    delete motionState_;
    delete shape_;
  }

  void Reset() {
    btTransform transform = body_->getCenterOfMassTransform();
    transform.setOrigin(btVector3(initial_position_.x, initial_position_.y,
                                  initial_position_.z));
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

  void Force(const glm::vec3 &i) {
    body_->activate(true);
    auto a = body_->getLinearVelocity();
    if (a.length() < FLAGS_max_speed) {
      body_->setLinearVelocity(a + btVector3(i.x, i.y, i.z));
    }
  }

  void Dump(State *state) {
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

private:
  btCollisionShape *shape_;
  btDefaultMotionState *motionState_;
  btRigidBody *body_;
  glm::vec3 initial_position_;
  float scale_;
};

class World {
  friend class GameServer;

public:
  World(glm::vec3 gravity) {
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
    dynamicsWorld_->addRigidBody(cube->body_);
    cubes_.push_back(cube);
  }

  void Player(Cube *cube) {
    players_.push_back(cube);
    Add(cube);
  }

  void Update(float deltaTime) {
    dynamicsWorld_->stepSimulation(deltaTime, 10);

    for (auto p : players_) {
      Katamari(p);
    }
  }

  void Katamari(Cube *player) {
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

  void Reset() {
    for (auto const cube : cubes_) {
      cube->Reset();
    }
    dynamicsWorld_->clearForces();
  }

  void Dump(std::vector<State> &state) {
    for (auto const cube : cubes_) {
      State s;
      cube->Dump(&s);
      state.emplace_back(s);
    }
  }

  void Dump(std::vector<QState> &state) {
    for (auto const cube : cubes_) {
      State s;
      cube->Dump(&s);
      state.emplace_back(s);
    }
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
  std::vector<Cube *> players_;
};

class GameServer {
public:
  GameServer()
      : server_(loop_), debug_server_(loop_), timer_(loop_), seq_(0),
        world_(glm::vec3(0, -20, 0)) {
    server_.OnReceive(
        [&](const uv_buf_t buf, const Addr addr) { OnReceive(buf, addr); });

    debug_server_.OnReceive([&](const uv_buf_t buf, const Addr addr) {
      OnDebugReceive(buf, addr);
    });

    for (int i = -FLAGS_cubes_count; i < FLAGS_cubes_count; ++i) {
      for (int j = -FLAGS_cubes_count; j < FLAGS_cubes_count; ++j) {
        world_.Add(new Cube(glm::vec3(i * 2, 0.5f, j * 2)));
      }
    }

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

    world_.Reset();
  }

  int ListenAndServe(const char *ip, int port) {
    debug_server_.Listen(ip, port + 1);
    // we use the same loop so as an side effect ListenAndServe will start debug
    // server as well
    return server_.ListenAndServe(ip, port);
  }

private:
  Frame &Next(int n) {
    // return Log()[n % Log().size()];
    return frame_;
  }

  void ApplyForce(const uv_buf_t &request, const Addr &addr) {
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

  void OnReceive(uv_buf_t request, Addr addr) {
    auto status = clients_.emplace(addr);
    if (status.second) {
      // printf("New client\n");
      NewPlayer(addr);
    }

    ApplyForce(request, addr);
  }

  void NewPlayer(Addr addr) {
    auto player =
        new Cube(glm::vec3(0, 15, 0), FLAGS_player_scale, FLAGS_player_mass);
    // world takes ownership of the player cube
    world_.Player(player);
    players_[addr] = player;
  }

  void OnTick() {
    Next(seq_);
    seq_++;

    if (!(seq_ % 6)) {
      size_t state_size = qframe_.size() * sizeof(QState);
      size_t total_size = state_size + sizeof(uint32_t);

      uv_buf_t response;
      response.base = Alloc(total_size);
      response.len = state_size;

      qframe_[0].interacting = seq_;

      memcpy(response.base, qframe_.data(), state_size);

      for (const auto &client : clients_) {
        server_.Send(client, &response);
      }
    }
  }

  void OnDebugReceive(uv_buf_t request, Addr addr) {
    debug_clients_.emplace(addr);
  }

  void OnDebugTick() {
    Frame &frame = Next(seq_);

    uv_buf_t response = {(char *)frame.data(), frame.size() * sizeof(State)};
    for (const auto &client : debug_clients_) {
      debug_server_.Send(client, &response);
    }
  }

  char *Alloc(int size) {
    static char slab[65536];
    CHECK(size < 65536);
    return slab;
  }

  uint64_t seq_;
  Loop loop_;
  UDPServer server_;
  Timer timer_;
  std::set<Addr> clients_;
  std::map<Addr, Cube *> players_;

  QFrame qframe_;
  Frame frame_;

  UDPServer debug_server_;
  std::set<Addr> debug_clients_;

  World world_;
};

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ReadLog(FLAGS_logfile);
  CHECK(Log().size() > 0);

  GameServer server;
  return server.ListenAndServe(FLAGS_server_addr.c_str(), FLAGS_server_port);
}
