#include "mj_sim_interface.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace wb_humanoid::mujoco_sim {
namespace {

constexpr double kRenderHz = 60.0;

mjModel* requireModel(mjModel* model) {
  if (model == nullptr) {
    throw std::runtime_error("MujocoSimInterface requires a non-null mjModel");
  }
  return model;
}

mjData* requireData(mjData* data) {
  if (data == nullptr) {
    throw std::runtime_error("MujocoSimInterface requires a non-null mjData");
  }
  return data;
}

int jointQposAddress(const mjModel* model, const std::string& jointName) {
  const int jointId = mj_name2id(model, mjOBJ_JOINT, jointName.c_str());
  if (jointId < 0) {
    throw std::runtime_error("MuJoCo joint not found: " + jointName);
  }
  return model->jnt_qposadr[jointId];
}

int jointDofAddress(const mjModel* model, const std::string& jointName) {
  const int jointId = mj_name2id(model, mjOBJ_JOINT, jointName.c_str());
  if (jointId < 0) {
    throw std::runtime_error("MuJoCo joint not found: " + jointName);
  }
  return model->jnt_dofadr[jointId];
}

std::vector<int> makeJointQposAddresses(const mjModel* model) {
  std::vector<int> addresses;
  addresses.reserve(g1JointNames().size());
  for (const auto& name : g1JointNames()) {
    addresses.emplace_back(jointQposAddress(model, name));
  }
  return addresses;
}

std::vector<int> makeJointDofAddresses(const mjModel* model) {
  std::vector<int> addresses;
  addresses.reserve(g1JointNames().size());
  for (const auto& name : g1JointNames()) {
    addresses.emplace_back(jointDofAddress(model, name));
  }
  return addresses;
}

std::vector<int> makeActuatorIds(const mjModel* model) {
  std::vector<int> ids;
  ids.reserve(g1JointNames().size());
  for (const auto& name : g1JointNames()) {
    const int actuatorId = mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
    if (actuatorId < 0) {
      throw std::runtime_error("MuJoCo actuator not found: " + name);
    }
    ids.emplace_back(actuatorId);
  }
  return ids;
}

std::vector<std::pair<double, double>> makeJointForceRanges(const mjModel* model) {
  std::vector<std::pair<double, double>> ranges;
  ranges.reserve(g1JointNames().size());
  for (const auto& name : g1JointNames()) {
    const int jointId = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (jointId < 0) {
      throw std::runtime_error("MuJoCo joint not found: " + name);
    }
    ranges.emplace_back(model->jnt_actfrcrange[2 * jointId], model->jnt_actfrcrange[2 * jointId + 1]);
  }
  return ranges;
}

}  // namespace

const std::vector<std::string>& g1JointNames() {
  static const std::vector<std::string> names = {
      "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
      "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
      "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
      "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
      "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
      "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
      "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
  return names;
}

Eigen::Quaterniond eulerZyxToQuat(double yaw, double pitch, double roll) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

Eigen::Vector3d quatToEulerZyx(const Eigen::Quaterniond& quat) {
  const Eigen::Matrix3d rotation = quat.normalized().toRotationMatrix();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  return Eigen::Vector3d(yaw, pitch, roll);
}

MujocoModelData::MujocoModelData(const std::string& mjcfPath, double timestep) {
  char error[1000] = "Could not load MuJoCo model";
  model_ = mj_loadXML(mjcfPath.c_str(), nullptr, error, sizeof(error));
  if (model_ == nullptr) {
    throw std::runtime_error("MuJoCo loadXML failed: " + std::string(error));
  }

  data_ = mj_makeData(model_);
  if (data_ == nullptr) {
    mj_deleteModel(model_);
    model_ = nullptr;
    throw std::runtime_error("MuJoCo data allocation failed for: " + mjcfPath);
  }

  model_->opt.timestep = timestep;
}

MujocoModelData::~MujocoModelData() {
  if (data_ != nullptr) {
    mj_deleteData(data_);
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
  }
}

MujocoSimInterface::MujocoSimInterface(mjModel* model, mjData* data)
    : model_(requireModel(model)),
      data_(requireData(data)),
      jointQposAddresses_(makeJointQposAddresses(model_)),
      jointDofAddresses_(makeJointDofAddresses(model_)),
      actuatorIds_(makeActuatorIds(model_)),
      jointForceRanges_(makeJointForceRanges(model_)) {}

void MujocoSimInterface::applyDefaultJointDamping(double damping) {
  for (int i = 6; i < model_->nv; ++i) {
    model_->dof_damping[i] = damping;
  }
}

RobotState MujocoSimInterface::readState() const {
  RobotState state;
  state.time = data_->time;
  state.basePosition = Eigen::Vector3d(data_->qpos[0], data_->qpos[1], data_->qpos[2]);
  state.baseQuat = Eigen::Quaterniond(data_->qpos[3], data_->qpos[4], data_->qpos[5], data_->qpos[6]).normalized();
  state.baseLinearVelocityWorld = Eigen::Vector3d(data_->qvel[0], data_->qvel[1], data_->qvel[2]);
  state.baseAngularVelocityLocal = Eigen::Vector3d(data_->qvel[3], data_->qvel[4], data_->qvel[5]);
  state.jointPosition.resize(jointQposAddresses_.size());
  state.jointVelocity.resize(jointDofAddresses_.size());
  for (size_t i = 0; i < jointQposAddresses_.size(); ++i) {
    state.jointPosition[i] = data_->qpos[jointQposAddresses_[i]];
    state.jointVelocity[i] = data_->qvel[jointDofAddresses_[i]];
  }
  return state;
}

void MujocoSimInterface::setFloatingBaseAndJoints(const Eigen::Vector3d& basePosition,
                                                  const Eigen::Quaterniond& baseQuat,
                                                  const std::vector<double>& jointPosition,
                                                  const std::vector<double>& jointVelocity) {
  if (jointPosition.size() != jointQposAddresses_.size()) {
    throw std::runtime_error("setFloatingBaseAndJoints received " + std::to_string(jointPosition.size()) +
                             " joint positions, expected " + std::to_string(jointQposAddresses_.size()));
  }
  if (!jointVelocity.empty() && jointVelocity.size() != jointDofAddresses_.size()) {
    throw std::runtime_error("setFloatingBaseAndJoints received " + std::to_string(jointVelocity.size()) +
                             " joint velocities, expected " + std::to_string(jointDofAddresses_.size()));
  }

  const Eigen::Quaterniond quat = baseQuat.normalized();
  data_->qpos[0] = basePosition.x();
  data_->qpos[1] = basePosition.y();
  data_->qpos[2] = basePosition.z();
  data_->qpos[3] = quat.w();
  data_->qpos[4] = quat.x();
  data_->qpos[5] = quat.y();
  data_->qpos[6] = quat.z();

  for (size_t i = 0; i < jointQposAddresses_.size(); ++i) {
    data_->qpos[jointQposAddresses_[i]] = jointPosition[i];
  }
  for (int i = 0; i < model_->nv; ++i) {
    data_->qvel[i] = 0.0;
  }
  if (!jointVelocity.empty()) {
    for (size_t i = 0; i < jointDofAddresses_.size(); ++i) {
      data_->qvel[jointDofAddresses_[i]] = jointVelocity[i];
    }
  }
  mj_forward(model_, data_);
}

std::vector<double> MujocoSimInterface::clampJointTorques(const std::vector<double>& torques) const {
  if (torques.size() != jointForceRanges_.size()) {
    throw std::runtime_error("clampJointTorques received " + std::to_string(torques.size()) +
                             " torques, expected " + std::to_string(jointForceRanges_.size()));
  }

  std::vector<double> clamped = torques;
  for (size_t i = 0; i < clamped.size(); ++i) {
    const auto [lo, hi] = jointForceRanges_[i];
    if (lo < hi) {
      clamped[i] = std::clamp(clamped[i], lo, hi);
    }
  }
  return clamped;
}

void MujocoSimInterface::setJointTorques(const std::vector<double>& torques) {
  if (torques.size() != actuatorIds_.size()) {
    throw std::runtime_error("setJointTorques received " + std::to_string(torques.size()) +
                             " torques, expected " + std::to_string(actuatorIds_.size()));
  }
  for (size_t i = 0; i < torques.size(); ++i) {
    data_->ctrl[actuatorIds_[i]] = torques[i];
  }
}

void MujocoSimInterface::step() {
  mj_step(model_, data_);
}

MujocoViewer::MujocoViewer(mjModel* model, mjData* data) : model_(model), data_(data) {
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&option_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);
}

MujocoViewer::~MujocoViewer() {
  if (window_ != nullptr) {
    glfwMakeContextCurrent(window_);
    mjr_freeContext(&context_);
    mjv_freeScene(&scene_);
    glfwDestroyWindow(window_);
    glfwTerminate();
  }
}

void MujocoViewer::initialize() {
  if (!glfwInit()) {
    throw std::runtime_error("GLFW init failed. Check DISPLAY/Wayland/X11 forwarding if you are running remotely.");
  }

  window_ = glfwCreateWindow(1280, 900, "centroidal_mpc_mujoco", nullptr, nullptr);
  if (window_ == nullptr) {
    glfwTerminate();
    throw std::runtime_error("GLFW window creation failed. No usable OpenGL display was found.");
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window_, this);
  glfwSetKeyCallback(window_, keyCallback);
  glfwSetCursorPosCallback(window_, cursorCallback);
  glfwSetMouseButtonCallback(window_, mouseButtonCallback);
  glfwSetScrollCallback(window_, scrollCallback);

  mjv_makeScene(model_, &scene_, 2000);
  mjr_makeContext(model_, &context_, mjFONTSCALE_150);

  camera_.type = mjCAMERA_FREE;
  camera_.lookat[0] = data_->qpos[0];
  camera_.lookat[1] = data_->qpos[1];
  camera_.lookat[2] = 0.75;
  camera_.distance = 3.0;
  camera_.azimuth = 135.0;
  camera_.elevation = -20.0;

  std::cout << "[centroidal_mpc_mujoco] viewer controls: Space pause, Right step, S real-time/fast, Esc quit."
            << std::endl;
}

bool MujocoViewer::shouldClose() const {
  return window_ != nullptr && glfwWindowShouldClose(window_);
}

bool MujocoViewer::simulationActive() const {
  return !paused_ || stepOnce_;
}

void MujocoViewer::finishStep() {
  if (stepOnce_) {
    stepOnce_ = false;
    paused_ = true;
  }
}

void MujocoViewer::render(bool force) {
  if (window_ == nullptr) {
    return;
  }

  const double now = glfwGetTime();
  if (!force && lastRenderWallTime_ >= 0.0 && now - lastRenderWallTime_ < 1.0 / kRenderHz) {
    glfwPollEvents();
    return;
  }
  lastRenderWallTime_ = now;

  if (camera_.type == mjCAMERA_FREE) {
    camera_.lookat[0] = 0.98 * camera_.lookat[0] + 0.02 * data_->qpos[0];
    camera_.lookat[1] = 0.98 * camera_.lookat[1] + 0.02 * data_->qpos[1];
    camera_.lookat[2] = 0.98 * camera_.lookat[2] + 0.02 * (data_->qpos[2] - 0.05);
  }

  mjrRect viewport{0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
  mjr_render(viewport, &scene_, &context_);

  char overlay[256];
  std::snprintf(overlay,
                sizeof(overlay),
                "time %.3f s\n%s%s",
                data_->time,
                paused_ ? "paused" : "running",
                runSlow_ ? "\nreal-time" : "\nfast");
  mjr_overlay(mjFONT_NORMAL,
              mjGRID_TOPLEFT,
              viewport,
              overlay,
              "Space pause\nRight step\nS speed\nEsc quit",
              &context_);

  glfwSwapBuffers(window_);
  glfwPollEvents();
}

void MujocoViewer::paceToRealTime() const {
  if (!runSlow_ || window_ == nullptr) {
    return;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(model_->opt.timestep));
}

void MujocoViewer::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
  (void)scancode;
  (void)mods;
  if (action != GLFW_PRESS) {
    return;
  }
  auto* viewer = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
  if (viewer == nullptr) {
    return;
  }
  if (key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  } else if (key == GLFW_KEY_SPACE) {
    viewer->paused_ = !viewer->paused_;
  } else if (key == GLFW_KEY_RIGHT) {
    viewer->paused_ = false;
    viewer->stepOnce_ = true;
  } else if (key == GLFW_KEY_S) {
    viewer->runSlow_ = !viewer->runSlow_;
  }
}

void MujocoViewer::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  (void)button;
  (void)mods;
  auto* viewer = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
  if (viewer == nullptr) {
    return;
  }
  viewer->leftMouse_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  viewer->middleMouse_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  viewer->rightMouse_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  if (action == GLFW_PRESS) {
    glfwGetCursorPos(window, &viewer->lastX_, &viewer->lastY_);
  }
}

void MujocoViewer::cursorCallback(GLFWwindow* window, double xpos, double ypos) {
  auto* viewer = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
  if (viewer == nullptr) {
    return;
  }

  if (!viewer->leftMouse_ && !viewer->middleMouse_ && !viewer->rightMouse_) {
    viewer->lastX_ = xpos;
    viewer->lastY_ = ypos;
    return;
  }

  const double dx = xpos - viewer->lastX_;
  const double dy = ypos - viewer->lastY_;
  viewer->lastX_ = xpos;
  viewer->lastY_ = ypos;

  int width = 1;
  int height = 1;
  glfwGetWindowSize(window, &width, &height);

  const bool shiftPressed =
      glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  mjtMouse action = mjMOUSE_MOVE_H;
  if (viewer->rightMouse_) {
    action = shiftPressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (viewer->leftMouse_) {
    action = shiftPressed ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else if (viewer->middleMouse_) {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(viewer->model_,
                 action,
                 dx / std::max(width, 1),
                 dy / std::max(height, 1),
                 &viewer->scene_,
                 &viewer->camera_);
}

void MujocoViewer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  (void)xoffset;
  auto* viewer = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
  if (viewer == nullptr) {
    return;
  }
  mjv_moveCamera(viewer->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &viewer->scene_, &viewer->camera_);
}

}  // namespace wb_humanoid::mujoco_sim
