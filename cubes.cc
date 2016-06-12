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

#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>
#include <glad/glad.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cstdio>
#include <cstdlib>
#include <vector>

#define GLSL(src) "#version 330 core\n" #src
#define UNUSED(x) (void)(x)

// Define Some Constants
int kWidth = 1280;
int kHeight = 800;

class ShaderBuilder {
public:
  ShaderBuilder() : vertex_shader_(0), fragment_shader_(0) {}

  ShaderBuilder &Vertex(const char *source) {
    GLuint shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    vertex_shader_ = shader;
    return *this;
  }

  ShaderBuilder &Fragment(const char *source) {
    GLuint shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    fragment_shader_ = shader;
    return *this;
  }

  GLuint Build() {
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader_);
    glAttachShader(program, fragment_shader_);
    glLinkProgram(program);

    glDeleteShader(vertex_shader_);
    vertex_shader_ = 0;

    glDeleteShader(fragment_shader_);
    fragment_shader_ = 0;

    return program;
  }

public:
  GLuint vertex_shader_;
  GLuint fragment_shader_;
};

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

class CubeShader {
public:
  CubeShader(GLuint program) {
    glUseProgram(program);

    object_color_location_ = glGetUniformLocation(program, "objectColor");
    light_color_location_ = glGetUniformLocation(program, "lightColor");
    light_position_location_ = glGetUniformLocation(program, "lightPos");
    model_location_ = glGetUniformLocation(program, "model");
    view_location_ = glGetUniformLocation(program, "view");
    projection_location_ = glGetUniformLocation(program, "projection");
  }

  void ObjectColor(const glm::vec3 &color) {
    glUniform3f(object_color_location_, color.x, color.y, color.z);
  }

  void LightColor(const glm::vec3 &color) {
    glUniform3f(light_color_location_, color.x, color.y, color.z);
  }

  void LightPosition(const glm::vec3 &position) {
    glUniform3f(light_position_location_, position.x, position.y, position.z);
  }

  void Model(const glm::mat4 &model) {
    glUniformMatrix4fv(model_location_, 1, GL_FALSE, glm::value_ptr(model));
  }

  void View(const glm::mat4 &view) {
    glUniformMatrix4fv(view_location_, 1, GL_FALSE, glm::value_ptr(view));
  }

  void Projection(const glm::mat4 &projection) {
    glUniformMatrix4fv(projection_location_, 1, GL_FALSE,
                       glm::value_ptr(projection));
  }

private:
  // fragment args
  GLuint object_color_location_;
  GLuint light_color_location_;
  GLuint light_position_location_;

  // vertex args
  GLuint model_location_;
  GLuint view_location_;
  GLuint projection_location_;
};

// clang-format off
GLfloat vertices[] = {
  -0.5f, -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f,
  0.5f,  -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f,
  0.5f,  0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f,
  0.5f,  0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f,
  -0.5f, 0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f,
  -0.5f, -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f,
  
  -0.5f, -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,
  0.5f,  -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,
  0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  -0.5f, 0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  -0.5f, -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,
  
  -0.5f, 0.5f,  0.5f,  -1.0f, 0.0f,  0.0f,
  -0.5f, 0.5f,  -0.5f, -1.0f, 0.0f,  0.0f,
  -0.5f, -0.5f, -0.5f, -1.0f, 0.0f,  0.0f,
  -0.5f, -0.5f, -0.5f, -1.0f, 0.0f,  0.0f,
  -0.5f, -0.5f, 0.5f,  -1.0f, 0.0f,  0.0f,
  -0.5f, 0.5f,  0.5f,  -1.0f, 0.0f,  0.0f,
  
  0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
  0.5f,  0.5f,  -0.5f, 1.0f,  0.0f,  0.0f,
  0.5f,  -0.5f, -0.5f, 1.0f,  0.0f,  0.0f,
  0.5f,  -0.5f, -0.5f, 1.0f,  0.0f,  0.0f,
  0.5f,  -0.5f, 0.5f,  1.0f,  0.0f,  0.0f,
  0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
  
  -0.5f, -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f,
  0.5f,  -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f,
  0.5f,  -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,
  0.5f,  -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,
  -0.5f, -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,
  -0.5f, -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f,
  
  -0.5f, 0.5f,  -0.5f, 0.0f,  1.0f,  0.0f,
  0.5f,  0.5f,  -0.5f, 0.0f,  1.0f,  0.0f,
  0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
  0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f, 0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f, 0.5f,  -0.5f, 0.0f,  1.0f,  0.0f
};
// clang-format on

class CubeModel {
public:
  CubeModel(GLuint program) : cs_(program), color_(1) {
    glGenVertexArrays(1, &VAO_);
    glGenBuffers(1, &VBO_);
  }

  ~CubeModel() {
    glDeleteVertexArrays(1, &VAO_);
    glDeleteBuffers(1, &VBO_);
  }

  void Data(GLfloat *vertices, size_t size) {
    glBindVertexArray(VAO_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_);

    glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat),
                          (GLvoid *)0);
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat),
                          (GLvoid *)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
  }

  void Color(const glm::vec3 &color) { color_ = color; }

  void Draw(const glm::mat4 &model, const glm::mat4 &view,
            const glm::mat4 &projection) {
    cs_.Model(model);
    cs_.View(view);
    cs_.Projection(projection);

    cs_.ObjectColor(color_);
    cs_.LightColor(glm::vec3(1.0f, 1.0f, 1.0f));
    cs_.LightPosition(glm::vec3(0.f, 25.f, 25.f));

    glBindVertexArray(VAO_);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
  }

private:
  GLuint VBO_;
  GLuint VAO_;
  CubeShader cs_;
  glm::vec3 color_;
};

class Cube {
public:
  Cube(glm::vec3 position, CubeModel *model, float scale = 1, float mass = 10)
      : cubeModel_(model), scale_(scale) {
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
    bodyCI.m_restitution = 0;
    body_ = new btRigidBody(bodyCI);

    // TODO(dzeromsk): find better api to start object inactive
    if (scale_ == 1) {
      body_->updateDeactivation(10.0f);
    }

    cubeModel_->Color(glm::vec3(1.0f, 1.0f, 1.0f));
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
    body_->setLinearVelocity(btVector3(0, 0, 0));
    body_->setAngularVelocity(btVector3(0, 0, 0));
    body_->clearForces();
    body_->activate(true);
    if (scale_ == 1) {
      body_->updateDeactivation(10.0f);
    }
  }

  btRigidBody *GetBody() { return body_; }

  void Draw(const glm::mat4 &view, const glm::mat4 &projection) {
    // move
    btTransform trans;
    body_->getMotionState()->getWorldTransform(trans);
    glm::mat4 model;
    trans.getOpenGLMatrix(glm::value_ptr(model));

    // scale
    glm::mat4 scaled = glm::scale(model, glm::vec3(scale_));

    // color
    if (body_->wantsSleeping()) {
      cubeModel_->Color(glm::vec3(1.0f, 1.0f, 1.0f));
    } else {
      cubeModel_->Color(glm::vec3(0.5f, 0.0f, 0.0f));
    }

    // draw
    cubeModel_->Draw(scaled, view, projection);
  }

  void Force(const glm::vec3 &i) {
    body_->activate(true);
    // body_->applyCentralImpulse(btVector3(i.x, i.y, i.z));
    auto a = body_->getLinearVelocity();
    if (a.length() < 18) {
      body_->setLinearVelocity(a + btVector3(i.x, i.y, i.z));
    }
  }

  glm::vec3 Position() {
    auto transform = body_->getCenterOfMassTransform();
    auto pos = transform.getOrigin();
    return glm::vec3(pos.x(), pos.y(), pos.z());
  }

private:
  btCollisionShape *shape_;
  btDefaultMotionState *motionState_;
  btRigidBody *body_;
  CubeModel *cubeModel_;
  glm::vec3 initial_position_;
  float scale_;
};

static Cube *cube = nullptr;

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
        btScalar magnitude = 1.0f / distanceSquared * 8e3f;
        cube->applyCentralForce(direction * magnitude);
      }
    }
  }

  void Draw(const glm::mat4 &view, const glm::mat4 &projection) {
    for (auto const cube : cubes_) {
      cube->Draw(view, projection);
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

static World *world = nullptr;
static GLFWwindow *window = nullptr;

void key_callback(GLFWwindow *window, int key, int scancode, int action,
                  int mode) {
  UNUSED(scancode);
  UNUSED(mode);

  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }

  if (key == GLFW_KEY_Q && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }

  if (key == GLFW_KEY_R && action == GLFW_PRESS) {
    world->Reset();
  }

  if (key == GLFW_KEY_W) {
    cube->Force(glm::vec3(0.f, 0.f, -1.f));
  }

  if (key == GLFW_KEY_S) {
    cube->Force(glm::vec3(0.f, 0.f, 1.f));
  }

  if (key == GLFW_KEY_A) {
    cube->Force(glm::vec3(-1.f, 0.f, 0.f));
  }

  if (key == GLFW_KEY_D) {
    cube->Force(glm::vec3(1.f, 0.f, 0.f));
  }

  if (key == GLFW_KEY_T && action == GLFW_PRESS) {
    static bool black = true;
    if (black) {
      glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
      black = false;
    } else {
      glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
      black = true;
    }
  }

  if (key == GLFW_KEY_F11 && action == GLFW_PRESS) {
    static bool fullscreen = false;
    static int w = 0, h = 0;
    if (fullscreen) {
      glfwSetWindowMonitor(window, nullptr, 0, 0, w, h, GLFW_DONT_CARE);
      fullscreen = false;
    } else {
      GLFWmonitor *monitor = glfwGetPrimaryMonitor();
      const GLFWvidmode *mode = glfwGetVideoMode(monitor);
      glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height,
                           mode->refreshRate);
      fullscreen = true;
      w = kWidth;
      h = kHeight;
    }
  }
}

void window_size_callback(GLFWwindow *window, int width, int height) {
  UNUSED(window);
  kWidth = width;
  kHeight = height;
  glViewport(0, 0, width, height);
}

int main(int argc, char *argv[]) {
  UNUSED(argc);
  UNUSED(argv);

  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4);

  window = glfwCreateWindow(kWidth, kHeight, "OpenGL", nullptr, nullptr);
  if (window == nullptr) {
    fprintf(stderr, "Failed to Create OpenGL Context");
    return EXIT_FAILURE;
  }

  glfwMakeContextCurrent(window);
  gladLoadGL();
  glfwSetKeyCallback(window, key_callback);
  glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSwapInterval(1);

  glViewport(0, 0, kWidth, kHeight);
  glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);

  GLuint shaderProgram =
      ShaderBuilder().Vertex(vertexSource).Fragment(fragmentSource).Build();

  CubeModel cm(shaderProgram);
  cm.Data(vertices, sizeof(vertices));

  gDeactivationTime = btScalar(1.);

  world = new World(glm::vec3(0, -20, 0));
  // for (int i = 0; i < 999; ++i) {
  // 	glm::vec3 pos = glm::linearRand(glm::vec3(-50.f, 0.5f, -50.f),
  // glm::vec3(50.f, 0.5f, 50.f));
  // 	Cube* c = new Cube(pos, &cm);
  // 	world->Add(c);
  // }

  for (int i = -15; i < 15; ++i) {
    for (int j = -15; j < 15; ++j) {
      glm::vec3 pos = glm::vec3(i * 2, 0.5f, j * 2);
      Cube *c = new Cube(pos, &cm);
      world->Add(c);
    }
  }

  cube = new Cube(glm::vec3(0, 15, 0), &cm, 4, 1e3f);
  world->Add(cube);

  // Deltatime
  GLfloat deltaTime = 0.0f; // Time between current frame and last frame
  GLfloat lastFrame = 0.0f; // Time of last frame

  // Camera/View transformation
  glm::mat4 view;
  glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

  int frames = 0;
  GLfloat lastUpdate = 0; // Time of last frame

  while (glfwWindowShouldClose(window) == false) {
    GLfloat currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    if (currentFrame - lastUpdate >= 1.0) {
      char buffer[256] = {0};
      snprintf(buffer, 255, "FPS: %d", frames);
      glfwSetWindowTitle(window, buffer);
      frames = 0;
      lastUpdate = currentFrame;
    }
    frames++;

    glfwPollEvents();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::mat4 projection = glm::perspective(
        45.0f, (GLfloat)kWidth / (GLfloat)kHeight, 0.1f, 100.0f);

    auto c = cube->Position();
    glm::mat4 view = glm::lookAt(c + glm::vec3(0.f, 15.f, 25.f), c, cameraUp);

    world->Update(deltaTime);
    world->Draw(view, projection);

    glfwSwapBuffers(window);
  }

  glfwTerminate();

  return EXIT_SUCCESS;
}
