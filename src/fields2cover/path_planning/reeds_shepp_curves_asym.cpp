//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#include <steering_functions/reeds_shepp_state_space/reeds_shepp_state_space.hpp>
#include <steering_functions/utilities/utilities.hpp>
#include "fields2cover/path_planning/steer_to_path.hpp"
#include "fields2cover/path_planning/reeds_shepp_curves_asym.h"

namespace f2c::pp {

namespace {

// Custom path generation with asymmetric turning radii
std::vector<steer::State> integrateAsym(
    const steer::State& start_state,
    const std::vector<steer::Control>& controls,
    double kappa_left,
    double kappa_right,
    double discretization) {

  std::vector<steer::State> path;
  steer::State state_curr, state_next;

  // Reserve capacity
  int n_states = 0;
  for (const auto& control : controls) {
    n_states += static_cast<int>(ceil(fabs(control.delta_s) / discretization));
  }
  path.reserve(n_states + 5);

  // Initialize current state
  state_curr.x = start_state.x;
  state_curr.y = start_state.y;
  state_curr.theta = start_state.theta;

  for (const auto& control : controls) {
    double delta_s = control.delta_s;
    double abs_delta_s = fabs(delta_s);
    double s_seg = 0.0;
    double integration_step = 0.0;

    // Determine actual kappa based on turn direction
    // Positive kappa = left turn, negative kappa = right turn
    double actual_kappa;
    if (fabs(control.kappa) < get_epsilon()) {
      actual_kappa = 0.0;  // Straight segment
    } else if (control.kappa > 0) {
      actual_kappa = kappa_left;  // Left turn
    } else {
      actual_kappa = -kappa_right;  // Right turn
    }

    // Push current state
    state_curr.kappa = actual_kappa;
    state_curr.d = sgn(delta_s);
    path.push_back(state_curr);

    int n = static_cast<int>(ceil(abs_delta_s / discretization));
    for (int i = 0; i < n; ++i) {
      // Calculate integration step
      s_seg += discretization;
      if (s_seg > abs_delta_s) {
        integration_step = discretization - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      } else {
        integration_step = discretization;
      }

      double d = sgn(delta_s);

      // Integrate ODE
      if (fabs(state_curr.kappa) > get_epsilon()) {
        // Circular arc
        end_of_circular_arc(
            state_curr.x, state_curr.y, state_curr.theta,
            state_curr.kappa, d, integration_step,
            &state_next.x, &state_next.y, &state_next.theta);
        state_next.kappa = state_curr.kappa;
        state_next.d = d;
      } else {
        // Straight line
        end_of_straight_line(
            state_curr.x, state_curr.y, state_curr.theta,
            d, integration_step,
            &state_next.x, &state_next.y);
        state_next.theta = state_curr.theta;
        state_next.kappa = 0.0;
        state_next.d = d;
      }

      path.push_back(state_next);
      state_curr = state_next;
    }
  }

  return path;
}

}  // anonymous namespace

F2CPath ReedsSheppCurvesAsym::createSimpleTurn(const F2CRobot& robot,
    double dist_start_pos, double start_angle, double end_angle) {

  // Get asymmetric curvatures
  double kappa_left = robot.getMaxCurvLeft();
  double kappa_right = robot.getMaxCurvRight();

  // Use the larger curvature (smaller radius) for path planning
  // This ensures the path is feasible for both turn directions
  double kappa_planning = std::max(kappa_left, kappa_right);

  steer::State start, end;

  start.x = 0.0;
  start.y = 0.0;
  start.theta = start_angle;
  start.kappa = 0.0;
  start.d = 0;

  end.x = dist_start_pos;
  end.y = 0.0;
  end.theta = end_angle;
  end.kappa = 0.0;
  end.d = 0;

  // Calculate path using symmetric Reeds-Shepp with planning curvature
  Reeds_Shepp_State_Space ss(kappa_planning, discretization);

  // Get controls from symmetric planner
  std::vector<steer::Control> controls = ss.get_controls(start, end);

  // Generate path with asymmetric radii
  std::vector<steer::State> path_states = integrateAsym(
      start, controls, kappa_left, kappa_right, discretization);

  return steerStatesToPath(path_states, robot.getTurnVel());
}

}  // namespace f2c::pp
