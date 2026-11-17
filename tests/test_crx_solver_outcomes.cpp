// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
/**
 * @file test_crx_solver_outcomes.cpp
 * @brief Tests for crx solver outcomes.
 */

#include "crx_canonical.h"
#include "crx_coordinates.h"
#include "crx_discovery.h"
#include "crx_joint_recovery.h"
#include "crx_joint_trig.h"
#include "crx_numerics.h"
#include "crx_robodk_adapter.h"
#include "crx_solver.h"
#include "crx_types.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <Eigen/LU>

namespace {
constexpr double kPi = 3.14159265358979323846;
using crx::IkStatus;
using crx::Vec6;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "Solver outcome: %s\n", message);
    std::exit(EXIT_FAILURE);
  }
}

auto Model() -> crx::CrxModelData {
  crx::CrxModelData model;
  model.base_transform.translation().z() = -245;
  model.joint_senses = {1, 1, -1, -1, -1, -1};
  model.dh_rows = {{{0, 0, 0, 245, false},
                    {-kPi / 2, 0, -kPi / 2, 0, false},
                    {0, 540, 0, 0, false},
                    {-kPi / 2, 0, 0, 540, false},
                    {kPi / 2, 0, 0, -150, false},
                    {-kPi / 2, 0, 0, 160, false}}};
  model.lower_limits_rad.setConstant(-2 * kPi);
  model.upper_limits_rad.setConstant(2 * kPi);
  return model;
}

auto Target(const crx::CrxModelData &model, const Vec6 &q) -> crx::PoseIsoRT {
  crx::PoseIsoRT pose;
  Require(crx::SolveFkIsometry(model, q, pose, nullptr, false) == 1,
          "construct target by FK");
  return pose;
}

auto SixteenSolutionModel() -> crx::CrxModelData {
  crx::CrxModelData model;
  model.base_transform.translation().z() = -0.5;
  model.joint_senses = {1, 1, -1, -1, -1, -1};
  model.dh_rows = {{{0, 0, 0, 0.5, false},
                    {-kPi / 2, 0, -kPi / 2, 0, false},
                    {0, 1, 0, 0, false},
                    {-kPi / 2, 0, 0, 0.6, false},
                    {kPi / 2, 0, 0, -0.5, false},
                    {-kPi / 2, 0, 0, 0.2, false}}};
  model.lower_limits_rad.setConstant(-kPi);
  model.upper_limits_rad.setConstant(kPi);
  return model;
}

auto SixteenSolutionTarget() -> crx::PoseIsoRT {
  crx::PoseIsoRT target = crx::PoseIsoRT::Identity();
  target.linear() << 3239.0 / 4059, 2182.0 / 4059, 1106.0 / 4059,
      -2378.0 / 4059, 3239.0 / 4059, 574.0 / 4059, -574.0 / 4059,
      -1106.0 / 4059, 3863.0 / 4059;
  target.translation() << 27.0 / 64, 15.0 / 32, 55.0 / 128;
  return target;
}

auto NumericalJacobianDeterminant(const crx::CrxModelData &model, const Vec6 &q)
    -> double {
  crx::PoseIsoRT pose;
  crx::JointPoseBuffer frames;
  Require(crx::SolveFkIsometry(model, q, pose, &frames, false) == 1,
          "Jacobian FK succeeds");
  Eigen::Matrix<double, 6, 6> jacobian;
  for (int joint = 0; joint < crx::kDofCount; ++joint) {
    const auto index = static_cast<std::size_t>(joint);
    const crx::Vec3 axis =
        model.joint_senses[index] * frames[index + 1].linear().col(2);
    jacobian.template block<3, 1>(0, joint) =
        axis.cross(pose.translation() - frames[index + 1].translation());
    jacobian.template block<3, 1>(3, joint) = axis;
  }
  return jacobian.determinant();
}

void CheckFibreIdentification() {
  const auto model = SixteenSolutionModel();
  const auto target = SixteenSolutionTarget();
  crx::canonical::WristCircle circle;
  Require(
      crx::canonical::PrepareWristCircle({1, 0.6, 0.2, 0.5}, target, circle) ==
          crx::canonical::PreparationStatus::Ready,
      "sixteen-target residual circle prepares");
  for (const double angle : {-2.2, -0.7, 0.3, 1.4, 2.8}) {
    constexpr double step = 1e-5;
    const double numerical =
        (crx::canonical::EvaluateResidual(circle, angle + step) -
         crx::canonical::EvaluateResidual(circle, angle - step)) /
        (2 * step);
    const double analytic =
        crx::canonical::EvaluateResidualDerivative(circle, angle);
    Require(std::abs(numerical - analytic) <=
                2e-7 * std::max(1.0, std::abs(numerical)),
            "wrist residual derivative agrees with central difference");
  }
  const auto result = crx::IdentifyIkIsometry(model, target, 64);
  Require(result.status == IkStatus::StrictCandidates,
          "sixteen-target fibre identification succeeds");
  Require(result.solutions.size() == 16,
          "sixteen-target yields all sixteen modulo-turn solutions");
  Require(!result.truncated, "sixteen-target result is not truncated");
  std::array<int, 64> root_multiplicity{};
  std::array<bool, 64> root_flip_zero{};
  std::array<bool, 64> root_flip_one{};
  for (std::size_t i = 0; i < result.solutions.size(); ++i) {
    const auto &solution = result.solutions[i];
    const auto fk = Target(model, solution.joints);
    Require((fk.matrix() - target.matrix()).norm() < 1e-8,
            "identified solution reconstructs target pose");
    Require(solution.regular && solution.aspect_sign != 0,
            "sixteen-target solutions are regular");
    Require(solution.wrist_address_valid,
            "sixteen-target has unambiguous wrist addresses");
    const double determinant =
        NumericalJacobianDeterminant(model, solution.joints);
    Require((determinant > 0 ? 1 : -1) == solution.aspect_sign,
            "closed-form aspect sign agrees with numerical Jacobian");
    const auto root = static_cast<std::size_t>(solution.wrist_address[0] * 8 +
                                               solution.wrist_address[1]);
    Require(root < root_multiplicity.size(), "wrist address is in range");
    ++root_multiplicity[root];
    if (solution.wrist_address[2] == 0) {
      root_flip_zero[root] = true;
    } else {
      root_flip_one[root] = true;
    }
    for (std::size_t j = 0; j < i; ++j) {
      Vec6 difference = solution.joints - result.solutions[j].joints;
      difference = difference.unaryExpr(
          [](double value) { return std::remainder(value, 2 * kPi); });
      Require(difference.norm() > 1e-5,
              "identified joint vectors are distinct modulo turns");
    }
  }
  int labelled_roots = 0;
  for (std::size_t i = 0; i < root_multiplicity.size(); ++i) {
    if (root_multiplicity[i] == 0) {
      continue;
    }
    ++labelled_roots;
    Require(root_multiplicity[i] == 2 && root_flip_zero[i] && root_flip_one[i],
            "each wrist root has both base-flip representatives");
  }
  Require(labelled_roots == 8, "sixteen-target has eight wrist roots");

  auto turn_model = model;
  turn_model.lower_limits_rad[0] = -3 * kPi;
  turn_model.upper_limits_rad[0] = 3 * kPi;
  const auto turn_result = crx::IdentifyIkIsometry(turn_model, target, 64);
  Require(turn_result.status == IkStatus::StrictCandidates,
          "turn-expanded fibre identification succeeds");
  bool found_turn_duplicate = false;
  for (std::size_t i = 0; i < turn_result.solutions.size(); ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      const auto &a = turn_result.solutions[i];
      const auto &b = turn_result.solutions[j];
      if (a.wrist_address_valid && b.wrist_address_valid &&
          a.wrist_address == b.wrist_address &&
          std::abs(std::abs(a.joints[0] - b.joints[0]) - 2 * kPi) < 1e-8) {
        found_turn_duplicate = true;
        break;
      }
    }
    if (found_turn_duplicate) {
      break;
    }
  }
  Require(found_turn_duplicate,
          "full-turn command variants retain their canonical wrist address");
}

void CheckJointTrig() {
  const double infinity = std::numeric_limits<double>::infinity();
  const auto check = [](const crx::JointTrig::Values &angles) {
    const crx::JointTrig trig(angles);
    for (std::size_t joint = 0; joint < trig.sine.size(); ++joint) {
      const double angle = angles.at(joint);
      Require(std::abs(trig.sine[joint] - std::sin(angle)) <=
                      8 * std::numeric_limits<double>::epsilon() &&
                  std::abs(trig.cosine[joint] - std::cos(angle)) <=
                      8 * std::numeric_limits<double>::epsilon(),
              "batched joint trig agrees with scalar reference");
      Require(std::abs(trig.sine[joint] * trig.sine[joint] +
                       trig.cosine[joint] * trig.cosine[joint] - 1) <=
                  8 * std::numeric_limits<double>::epsilon(),
              "batched joint trig stays on the unit circle");
    }
  };
  // Mix lanes and include reduction boundaries, signed zero, and large turns.
  for (const double center :
       {0.0, kPi / 2, -kPi, 2 * kPi, 15.0, 1e6, 1e13, 1e14, 1e20, 1e300}) {
    const crx::JointTrig::Values angles{center,
                                        -center,
                                        std::nextafter(center, infinity),
                                        std::nextafter(center, -infinity),
                                        .375,
                                        -1.125};
    check(angles);
  }
  // Exercise every lane over the vector interval, including both sides of
  // quadrant boundaries where the result switches approximating polynomial.
  for (int sample = -1000; sample <= 1000; ++sample) {
    crx::JointTrig::Values angles{};
    for (std::size_t joint = 0; joint < angles.size(); ++joint) {
      angles.at(joint) = .014 * sample + .031 * static_cast<double>(joint);
    }
    check(angles);
  }
  for (int quadrant = -9; quadrant <= 9; ++quadrant) {
    const double angle = quadrant * kPi / 2;
    const crx::JointTrig::Values angles{angle,
                                        -angle,
                                        std::nextafter(angle, infinity),
                                        std::nextafter(angle, -infinity),
                                        angle + 1e-10,
                                        angle - 1e-10};
    check(angles);
  }
  crx::JointTrig::Values zeros{};
  zeros[1] = -0.0;
  const crx::JointTrig trig(zeros);
  Require(!std::signbit(trig.sine[0]) && std::signbit(trig.sine[1]) &&
              trig.cosine[0] == 1 && trig.cosine[1] == 1,
          "batched joint trig preserves signed zero");
  const crx::JointTrig::Values exceptional{
      infinity, -infinity, std::numeric_limits<double>::quiet_NaN(),
      .375,     -1.125,    0.0};
  const crx::JointTrig nonfinite(exceptional);
  for (std::size_t joint = 0; joint < 3; ++joint) {
    Require(std::isnan(nonfinite.sine[joint]) &&
                std::isnan(nonfinite.cosine[joint]),
            "exceptional trig inputs preserve scalar classification");
  }
  Require(nonfinite.sine[3] == std::sin(exceptional[3]) &&
              nonfinite.cosine[4] == std::cos(exceptional[4]),
          "exceptional trig lanes do not corrupt finite lanes");
}

void CheckCrxForwardGeometry() {
  auto model = Model();
  model.base_transform.linear() =
      Eigen::AngleAxisd(.2, crx::Vec3::UnitZ()).toRotationMatrix();
  model.tool_transform.translation() = crx::Vec3(3, -2, 1);
  Vec6 q;
  q << .4, -.7, .1, .6, -.9, -1.2;
  for (const double offset : {0.0, 3.0}) {
    model.dh_rows[4].d += offset;
    crx::PoseIsoRT actual;
    crx::JointPoseBuffer joints;
    Require(crx::SolveFkIsometry(model, q, actual, &joints, false) == 1,
            "CRX family geometry supports FK and CAD frames");
    crx::PoseIsoRT expected = model.base_transform;
    Require(joints[0].matrix().isApprox(expected.matrix(), 1e-12),
            "joint buffer starts at the model base");
    for (std::size_t i = 0; i < model.dh_rows.size(); ++i) {
      const auto &row = model.dh_rows[i];
      const double theta = row.theta0_rad + q[static_cast<Eigen::Index>(i)] *
                                                model.joint_senses[i];
      const double d = row.d;
      expected = expected * Eigen::Translation3d(row.a, 0, 0) *
                 Eigen::AngleAxisd(row.alpha_rad, crx::Vec3::UnitX()) *
                 Eigen::Translation3d(0, 0, d) *
                 Eigen::AngleAxisd(theta, crx::Vec3::UnitZ());
      Require((joints[i + 1].matrix() - expected.matrix()).norm() < 1e-10,
              "every joint frame matches independent modified-DH factors");
    }
    expected = expected * model.tool_transform;
    Require((actual.matrix() - expected.matrix()).norm() < 1e-10,
            "FK uses this call's geometry and tool transform");
  }
}

auto IndependentDhPose(const crx::CrxModelData &model, const Vec6 &command)
    -> crx::PoseIsoRT {
  crx::PoseIsoRT pose = model.base_transform;
  for (std::size_t i = 0; i < model.dh_rows.size(); ++i) {
    const auto &row = model.dh_rows.at(i);
    const double motion =
        model.joint_senses.at(i) * command[static_cast<Eigen::Index>(i)];
    pose = pose * Eigen::Translation3d(row.a, 0, 0) *
           Eigen::AngleAxisd(row.alpha_rad, crx::Vec3::UnitX()) *
           Eigen::Translation3d(0, 0, row.d) *
           Eigen::AngleAxisd(row.theta0_rad, crx::Vec3::UnitZ()) *
           Eigen::AngleAxisd(motion, crx::Vec3::UnitZ());
  }
  return pose * model.tool_transform;
}

auto DifferentiatedDeterminant(const crx::CrxModelData &model,
                               const Vec6 &command) -> double {
  constexpr double step = 1e-5;
  const auto centre = IndependentDhPose(model, command);
  Eigen::Matrix<double, 6, 6> jacobian;
  for (int axis = 0; axis < crx::kDofCount; ++axis) {
    Vec6 plus = command, minus = command;
    plus[axis] += step;
    minus[axis] -= step;
    const auto forward = IndependentDhPose(model, plus);
    const auto backward = IndependentDhPose(model, minus);
    jacobian.block<3, 1>(0, axis) =
        (forward.translation() - backward.translation()) / (2 * step);
    const crx::Mat3 angular = (forward.linear() - backward.linear()) *
                              centre.linear().transpose() / (2 * step);
    jacobian.block<3, 1>(3, axis) << angular(2, 1), angular(0, 2),
        angular(1, 0);
  }
  return jacobian.determinant();
}

void CheckCanonicalCoordinates() {
  const auto nominal = SixteenSolutionModel();
  const auto target = SixteenSolutionTarget();
  const auto reference = crx::IdentifyIkIsometry(nominal, target, 64);
  Require(reference.solutions.size() == 16, "canonical reference has 16 roots");
  for (const double signed_link : {0.6, -0.6}) {
    for (const double scale : {0.001, 1.0, 1000.0}) {
      auto model = nominal;
      model.dh_rows[3].d = signed_link;
      for (auto &row : model.dh_rows) {
        row.a *= scale;
        row.d *= scale;
      }
      model.base_transform.translation().z() *= scale;
      crx::CanonicalCommandMap map;
      // Mixed senses, nonzero offsets and whole-turn offset equivalence.
      for (int axis = 0; axis < crx::kDofCount; ++axis) {
        const auto i = static_cast<std::size_t>(axis);
        map.diagonal[axis] = axis % 2 == 0 ? -1.0 : 1.0;
        map.offset[axis] = 0.07 * static_cast<double>(axis + 1);
        model.joint_senses.at(i) *= map.diagonal[axis];
        model.dh_rows.at(i).theta0_rad +=
            nominal.joint_senses.at(i) * map.offset[axis] + 2 * kPi;
      }
      // The desired command box is the inverse image of the canonical box.
      model.lower_limits_rad =
          map.ToCommand(nominal.lower_limits_rad)
              .cwiseMin(map.ToCommand(nominal.upper_limits_rad));
      model.upper_limits_rad =
          map.ToCommand(nominal.lower_limits_rad)
              .cwiseMax(map.ToCommand(nominal.upper_limits_rad));
      model.base_transform.linear() =
          Eigen::AngleAxisd(0.31, crx::Vec3::UnitX()).toRotationMatrix();
      model.tool_transform.translation() << 0.03 * scale, -0.02 * scale,
          0.01 * scale;
      const auto canonical = reference.solutions.front().joints;
      Vec6 command = map.ToCommand(canonical);
      Require(map.FromCommand(command).isApprox(canonical, 1e-13),
              "affine command map and inverse agree");
      // Signed b uses a separate target from the same independent chain.
      const auto desired = IndependentDhPose(model, command);
      Require((Target(model, command).matrix() - desired.matrix()).norm() <
                  1e-10 * std::max(1.0, scale),
              "mixed-sense offset FK agrees with independent DH products");
      const auto identified = crx::IdentifyIkIsometry(model, desired, 64);
      Require(identified.status == IkStatus::StrictCandidates &&
                  !identified.solutions.empty(),
              "signed and mixed-sense identification succeeds");
      for (const auto &solution : identified.solutions) {
        const auto actual = IndependentDhPose(model, solution.joints);
        Require((actual.matrix() - desired.matrix()).norm() <
                    1e-7 * std::max(1.0, scale),
                "mapped identification commands pass independent FK");
        if (solution.regular) {
          const auto determinant =
              DifferentiatedDeterminant(model, solution.joints);
          const double map_sign = map.diagonal.prod();
          Require(
              (determinant * map_sign > 0 ? 1 : -1) == solution.aspect_sign,
              "canonical aspect agrees with differentiated command Jacobian");
        }
      }
      const auto strict = crx::SolveIkIsometry(model, desired, &command, 1);
      Require(strict.status == IkStatus::StrictSeed &&
                  strict.solutions.front().isApprox(command, 1e-12),
              "mapped strict seed preserves command coordinates");
      Vec6 nearby = command + Vec6::Constant(1e-5);
      const auto local = crx::SolveIkIsometry(model, desired, &nearby, 1);
      Require(!local.solutions.empty(),
              "nearby discovery consumes canonical seed coordinates");
      const auto full = crx::SolveIkIsometry(model, desired, nullptr, 64);
      Require(full.status == IkStatus::StrictCandidates &&
                  !full.solutions.empty(),
              "unseeded discovery returns mapped command lifts");
      model.lower_limits_rad = command - Vec6::Constant(0.01);
      model.upper_limits_rad = command + Vec6::Constant(0.01);
      const auto limited = crx::SolveIkIsometry(model, desired, nullptr, 64);
      Require(limited.status == IkStatus::StrictCandidates &&
                  limited.solutions.size() == 1,
              "mapped narrow limits retain the independent witness");
    }
  }
}

void CheckLargeEncoderOffsets() {
  Vec6 command;
  command << 0.4, -0.7, 0.1, 0.6, -0.9, -1.2;
  for (const double offset : {-1e20, -1e12, 1e12, 1e20}) {
    for (std::size_t axis = 0; axis < crx::kDofCount; ++axis) {
      auto model = Model();
      model.dh_rows.at(axis).theta0_rad = offset;
      const auto expected = IndependentDhPose(model, command);
      Require((Target(model, command).matrix() - expected.matrix()).norm() <
                  1e-9,
              "large encoder zeros retain their actual periodic rotation");
      const auto seeded = crx::SolveIkIsometry(model, expected, &command, 1);
      Require(seeded.status == IkStatus::StrictSeed &&
                  seeded.solutions.front().isApprox(command, 1e-12),
              "large encoder offsets preserve strict command seeds");
    }
  }
}

void CheckUnsupportedModels() {
  const Vec6 command = Vec6::Constant(.3);
  const auto target = Target(Model(), command);
  const auto rejected = [&](const crx::CrxModelData &model) {
    auto pose = crx::PoseIsoRT::Identity();
    crx::JointPoseBuffer frames;
    frames.fill(pose);
    crx::ArmPosture posture{true, true, true};
    Require(crx::SolveFkIsometry(model, command, pose, &frames, false) == -1 &&
                crx::SolveFkIsometry(model, command, pose, nullptr, true) == -1,
            "FK and CAD reject unsupported models");
    Require(pose.matrix().isIdentity() && frames.back().matrix().isIdentity(),
            "model rejection preserves FK outputs");
    Require(!crx::ClassifyArmPosture(model, command, posture) && posture.rear &&
                posture.lower_arm && posture.flip,
            "configuration rejects unsupported models without writing outputs");
    const auto ik = crx::SolveIkIsometry(model, target, &command, 1);
    Require(ik.status == IkStatus::UnsupportedModel && ik.solutions.empty(),
            "IK rejects unsupported models before seed acceptance");
  };
  for (std::size_t axis = 0; axis < crx::kDofCount; ++axis) {
    auto model = Model();
    model.dh_rows[axis].is_prismatic = true;
    rejected(model);
    model = Model();
    model.dh_rows[axis].alpha_rad += 1e-8;
    rejected(model);
    model = Model();
    model.dh_rows[axis].a = axis == 2 ? -1 : 1;
    rejected(model);
  }
  auto model = Model();
  model.dh_rows[1].d = 1;
  rejected(model);
  model = Model();
  model.dh_rows[4].d = 0;
  rejected(model);
}

void CheckMappings() {
  for (const auto status : {IkStatus::StrictCandidates, IkStatus::StrictSeed,
                            IkStatus::StrictLocal, IkStatus::ApproximateSeed}) {
    crx::IkResult result;
    result.status = status;
    Require(crx::IkResultToRoboDkReturn(result) == -1,
            "empty success rejected");
    result.solutions.push_back(Vec6::Zero());
    Require(crx::IkResultToRoboDkReturn(result) == 1, "success count");
    result.truncated = true;
    Require(crx::IkResultToRoboDkReturn(result) == 1,
            "truncation remains success");
  }
  for (const auto status :
       {IkStatus::NoCandidate, IkStatus::NoOutputRequested,
        IkStatus::InvalidInput, IkStatus::UnsupportedModel,
        IkStatus::Unresolved, IkStatus::NumericalRangeFailure,
        IkStatus::WorkLimit}) {
    crx::IkResult result;
    result.status = status;
    const int expected =
        status == IkStatus::NoCandidate || status == IkStatus::NoOutputRequested
            ? 0
            : -1;
    Require(crx::IkResultToRoboDkReturn(result) == expected,
            "failure/empty mapping");
  }
}

void CheckBaseBranchCoverage() {
  for (const double forearm : {540.0, 760.0}) {
    auto model = Model();
    model.dh_rows[3].d = forearm;
    Vec6 command;
    command << .4, -.7, .1, .6, -.9, -1.2;
    const auto target = Target(model, command);
    // Independent command-coordinate base symmetry, for equal and unequal arms.
    Vec6 dual = command;
    dual[0] += kPi;
    dual[1] *= -1;
    dual[2] = kPi - dual[2];
    dual[3] -= kPi;
    const auto full = crx::SolveIkIsometry(model, target, nullptr, 4096);
    Require(full.status == IkStatus::StrictCandidates && !full.truncated,
            "enumerate both base branches without a seed");
    for (const Vec6 &branch : {command, dual}) {
      bool found = false;
      for (const auto &solution : full.solutions) {
        const Vec6 difference = (solution - branch).unaryExpr([](double angle) {
          return std::remainder(angle, 2 * kPi);
        });
        found = found || difference.cwiseAbs().maxCoeff() < 1e-7;
      }
      Require(found, "both recovered base branches reach final output");
      // Force each branch independently, including a nonprincipal J3 turn.
      Vec6 lifted = branch;
      lifted[2] += 2 * kPi;
      model.lower_limits_rad = lifted.array() - .001;
      model.upper_limits_rad = lifted.array() + .001;
      const auto restricted = crx::SolveIkIsometry(model, target, nullptr, 64);
      Require(restricted.status == IkStatus::StrictCandidates &&
                  restricted.solutions.size() == 1 &&
                  (restricted.solutions[0] - lifted).cwiseAbs().maxCoeff() <
                      1e-7,
              "limits retain either base branch in command coordinates");
    }
  }
}

void CheckSolves() {
  auto model = Model();
  Vec6 q;
  q << .4, -.7, .1, .6, -.9, -1.2;
  const auto target = Target(model, q);
  const auto full = crx::SolveIkIsometry(model, target, &q, 4096);
  Require(full.status == IkStatus::StrictCandidates && !full.truncated &&
              !full.used_singular_representatives && full.solutions.size() > 1,
          "strict finite commands");
  const auto limited = crx::SolveIkIsometry(model, target, &q, 1);
  Require(limited.status == IkStatus::StrictSeed && !limited.truncated &&
              limited.solutions.size() == 1 &&
              limited.solutions[0].isApprox(q, crx::numerics::kEpsilon),
          "single-output exact seed returns without discovery");
  const Vec6 nearby = q.array() + .005;
  const auto tracked = crx::SolveIkIsometry(model, target, &nearby, 1);
  Require(tracked.status == IkStatus::StrictLocal && !tracked.truncated &&
              tracked.solutions.size() == 1 &&
              (Target(model, tracked.solutions[0]).matrix() - target.matrix())
                      .norm() < crx::numerics::kSolutionPositionToleranceMm,
          "nearby seed follows an algebraic branch with strict FK");
  const auto ranked = crx::SolveIkIsometry(model, target, &q, 2);
  Require(ranked.status == IkStatus::StrictCandidates && ranked.truncated &&
              ranked.solutions.size() == 2 &&
              ranked.solutions[0].isApprox(full.solutions[0],
                                           crx::numerics::kEpsilon),
          "multiple outputs keep ranked prefix and truncation metadata");
  model.lower_limits_rad.setConstant(-100 * kPi);
  model.upper_limits_rad.setConstant(100 * kPi);
  const auto work = crx::SolveIkIsometry(model, target, &q, 2);
  Require(work.status == IkStatus::WorkLimit && work.solutions.empty(),
          "work failure does not publish a seed or partial results");
  model.lower_limits_rad.setConstant(-1e20);
  model.upper_limits_rad.setConstant(1e20);
  const auto range = crx::SolveIkIsometry(model, target, &q, 2);
  Require(range.status == IkStatus::NumericalRangeFailure &&
              range.solutions.empty(),
          "command integer range failure is distinct from work exhaustion");
}

void CheckBoundaryStatusMetadata() {
  const auto model = Model();
  Vec6 seed;
  seed << 0, kPi / 2, kPi / 2, 0, 0, 0;
  for (const double delta : {0.00005, 0.005}) {
    auto target = Target(model, seed);
    target.translation().x() += delta;
    const auto result = crx::SolveIkIsometry(model, target, &seed, 64);
    Require(
        result.status == (delta < crx::numerics::kSolutionPositionToleranceMm
                              ? IkStatus::StrictCandidates
                              : IkStatus::Unresolved),
        "boundary correction must not mask unresolved discovery with a seed");
  }
}

void CheckSingularRepresentatives() {
  const auto model = Model();
  Vec6 seed;
  seed << 0, 0, kPi / 2, 0, 0, 0;
  const auto target = Target(model, seed);
  const auto result = crx::SolveIkIsometry(model, target, nullptr, 4096);
  Require(result.status == IkStatus::StrictCandidates &&
              !result.solutions.empty(),
          "vertical arm representatives are supported");
  Require(result.used_singular_representatives,
          "canonical free-base policy propagated");
}

void CheckPhaseDomains() {
  const auto model = Model();
  Vec6 command;
  command << .4, -.7, .1, .6, -.9, -1.2;
  const auto target = Target(model, command);
  const crx::canonical::Lengths lengths{540, 540, 160, 150};
  const crx::canonical::PoseTolerance tolerance{
      crx::numerics::kSolutionPositionToleranceMm,
      crx::numerics::kSolutionAngleToleranceRad};
  const auto full =
      crx::canonical::DiscoverJointCandidates(lengths, target, tolerance);
  Require(full.status == crx::canonical::RecoveryStatus::Candidates,
          "domain reference discovers regular target");
  for (const int axis : {2, 5}) {
    for (const double turns : {-2.0, 0.0, 2.0}) {
      Vec6 lower = Vec6::Constant(-2 * kPi), upper = -lower;
      lower[axis] = command[axis] + turns * 2 * kPi - .01;
      upper[axis] = command[axis] + turns * 2 * kPi + .01;
      crx::canonical::DiscoveryOptions options;
      options.lower_limits = &lower;
      options.upper_limits = &upper;
      const auto filtered = crx::canonical::DiscoverJointCandidates(
          lengths, target, tolerance, options);
      Require(filtered.status == crx::canonical::RecoveryStatus::Candidates &&
                  filtered.pruned_nominations > 0 &&
                  filtered.count < full.count,
              "each restricted J3/J6 domain prunes geometric nominations");
      for (std::size_t i = 0; i < full.count; ++i) {
        const auto &expected = full.candidates[i].joints;
        const double delta =
            std::remainder(expected[axis] - command[axis], 2 * kPi);
        if (std::abs(delta) > .01) {
          continue;
        }
        bool found = false;
        for (std::size_t j = 0; j < filtered.count; ++j) {
          const crx::Vec6 difference =
              (expected - filtered.candidates[j].joints)
                  .unaryExpr([](double angle) {
                    return std::remainder(angle, 2 * kPi);
                  });
          found = found || difference.isZero(1e-9);
        }
        Require(found, "domain sieve retains every feasible reference posture");
      }
    }
  }
  crx::canonical::DiscoveryOptions invalid;
  invalid.lower_limits = &command;
  Require(crx::canonical::DiscoverJointCandidates(lengths, target, tolerance,
                                                  invalid)
                  .status == crx::canonical::RecoveryStatus::InvalidInput,
          "partial domain box is invalid");
}
} // namespace

auto main() -> int {
  CheckFibreIdentification();
  CheckJointTrig();
  CheckCrxForwardGeometry();
  CheckCanonicalCoordinates();
  CheckLargeEncoderOffsets();
  CheckUnsupportedModels();
  CheckMappings();
  CheckBaseBranchCoverage();
  CheckSolves();
  CheckBoundaryStatusMetadata();
  CheckSingularRepresentatives();
  CheckPhaseDomains();
  std::puts("Solver outcome and adapter checks passed");
  return EXIT_SUCCESS;
}
