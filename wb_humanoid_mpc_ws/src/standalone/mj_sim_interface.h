#pragma once

#include <mujoco/mujoco.h>

#include <Eigen/Geometry>

#include <string>
#include <utility>
#include <vector>

struct GLFWwindow;

namespace wb_humanoid::mujoco_sim {

constexpr double kDefaultSimDt = 0.0005;

const std::vector<std::string>& g1JointNames();

Eigen::Quaterniond eulerZyxToQuat(double yaw, double pitch, double roll);
Eigen::Vector3d quatToEulerZyx(const Eigen::Quaterniond& quat);

struct RobotState {
  double time = 0.0;
  Eigen::Vector3d basePosition = Eigen::Vector3d::Zero();
  Eigen::Quaterniond baseQuat = Eigen::Quaterniond::Identity();
  Eigen::Vector3d baseLinearVelocityWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d baseAngularVelocityLocal = Eigen::Vector3d::Zero();
  std::vector<double> jointPosition;
  std::vector<double> jointVelocity;
};

class MujocoModelData {
 public:
  explicit MujocoModelData(const std::string& mjcfPath, double timestep = kDefaultSimDt);
  ~MujocoModelData();

  MujocoModelData(const MujocoModelData&) = delete;
  MujocoModelData& operator=(const MujocoModelData&) = delete;

  mjModel* model() const { return model_; }
  mjData* data() const { return data_; }

 private:
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
};

class MujocoSimInterface {
 public:
  MujocoSimInterface(mjModel* model, mjData* data);

  mjModel* model() const { return model_; }
  mjData* data() const { return data_; }
  double time() const { return data_->time; }

  const std::vector<std::pair<double, double>>& jointForceRanges() const { return jointForceRanges_; }

  void applyDefaultJointDamping(double damping);
  RobotState readState() const;
  void setFloatingBaseAndJoints(const Eigen::Vector3d& basePosition,
                                const Eigen::Quaterniond& baseQuat,
                                const std::vector<double>& jointPosition,
                                const std::vector<double>& jointVelocity = {});
  std::vector<double> clampJointTorques(const std::vector<double>& torques) const;
  void setJointTorques(const std::vector<double>& torques);
  void step();

 private:
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::vector<int> jointQposAddresses_;
  std::vector<int> jointDofAddresses_;
  std::vector<int> actuatorIds_;
  std::vector<std::pair<double, double>> jointForceRanges_;
};

class MujocoViewer {
 public:
  MujocoViewer(mjModel* model, mjData* data);
  ~MujocoViewer();

  MujocoViewer(const MujocoViewer&) = delete;
  MujocoViewer& operator=(const MujocoViewer&) = delete;

  void initialize();
  bool shouldClose() const;
  bool simulationActive() const;
  void finishStep();
  void render(bool force = false);
  void paceToRealTime() const;

 private:
  static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
  static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
  static void cursorCallback(GLFWwindow* window, double xpos, double ypos);
  static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  GLFWwindow* window_ = nullptr;
  mjvCamera camera_;
  mjvOption option_;
  mjvScene scene_;
  mjrContext context_;
  bool paused_ = false;
  bool stepOnce_ = false;
  bool runSlow_ = true;
  bool leftMouse_ = false;
  bool middleMouse_ = false;
  bool rightMouse_ = false;
  double lastX_ = 0.0;
  double lastY_ = 0.0;
  double lastRenderWallTime_ = -1.0;
};

}  // namespace wb_humanoid::mujoco_sim
