/* robot.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>
#include <memory>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include "drake/systems/primitives/vector_log_sink.h"

#include "robot.h"

State::State(StateValue& value, double time):
  value_{},
  time_{time} {
  this->set_value(value);
}

void State::set_value(StateValue& value) {
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (i == 2) {
      // Round up the angle if larger than 2 Pi
      value_[i] = fmod(value[i], 2 * M_PI);
    } else {
      value_[i] = value[i];
    }
  }
}

StateValue State::get_value() {
  return value_;
}

Coord2D State::get_coord() {
  return std::make_pair(value_[0], value_[1]);
}

double State::get_time() {
  return time_;
}

namespace robot_dynamic {
  HalfCarSystem::HalfCarSystem() {
    DeclareContinuousState(STATE_NUM);  // One state variable.
    DeclareVectorOutputPort("state", drake::systems::BasicVector<double>(STATE_NUM),
                            &HalfCarSystem::CopyStateOut);
  }

  void HalfCarSystem::DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const {
    const double x = context.get_continuous_state()[0];
    const double y = context.get_continuous_state()[1];
    const double t = context.get_continuous_state()[2];
    const double v = context.get_continuous_state()[3];
    const double w = context.get_continuous_state()[4];
    const double a = context.get_continuous_state()[5];
    const double r = context.get_continuous_state()[6];

    const double xdot = v * cos(t);
    const double ydot = v * sin(t);
    const double tdot = w;
    const double vdot = a;
    const double wdot = r;

    (*derivatives)[0] = xdot;
    (*derivatives)[1] = ydot;
    (*derivatives)[2] = tdot;

    // The linear velocity constraint
    // Set linear acceleration to zero
    // if the acc value will cause linear velocity over bound
    if ((v > V_MAX * 0.95 && a > 0) ||
        (v < V_MIN * 0.95 && a < 0)) {
      (*derivatives)[3] = 0;
    } else {
      (*derivatives)[3] = vdot;
    }

    // The angular velocity constraint
    // Set angular velocity to zero
    // if the angular acc value will cause angular velocity over bound
    if ((w > W_MAX * 0.95 && r > 0) ||
        (w < W_MIN * 0.95 && r < 0)) {
      (*derivatives)[4] = 0;
    } else {
      (*derivatives)[4] = wdot;
    }
  }

  void HalfCarSystem::CopyStateOut(const drake::systems::Context<double>& context,
                                   drake::systems::BasicVector<double>* output) const {
    // The output of this system are the states
    for (int i = 0; i < context.get_continuous_state().size(); ++i) {
      (*output)[i] = context.get_continuous_state()[i];
    }
  }

  std::vector<State> forward_simulation(
      State& init_state,
      double epsilon,
      double delta,
      double boundary_time) {
    drake::systems::DiagramBuilder<double> builder;
    auto system = builder.AddSystem(std::make_unique<HalfCarSystem>());
    auto logger = LogVectorOutput(system->get_output_port(0), &builder);
    auto diagram = builder.Build();

    auto context = diagram->CreateDefaultContext();

    Eigen::VectorXd v(STATE_NUM);

    // Set initial state
    for (std::size_t i = 0; i < STATE_NUM; ++i) {
      v(i) = init_state.get_value()[i];
    }

    context->SetContinuousState(v);

    drake::systems::Simulator<double> simulator(*diagram, std::move(context));

    // Start forward simulation
    simulator.AdvanceTo(boundary_time);
    auto log = logger->FindLog(simulator.get_context());

    std::vector<State> trajectory;

    double segment_length = 0.0;
    double total_length = 0.0;
    double prev_time = log.sample_times()[0];
    double prev_v = log.data()(3);

    // Trim the trajectory and leave waypoints that fits
    // the rrt delta length and epsilon length
    for (int i = 1; i < log.sample_times().size(); ++i) {
      auto v = log.data()(i * STATE_NUM + 3);
      double curr_time = log.sample_times()[i];
      double duration = curr_time - prev_time;
      segment_length += abs(0.5 * (v + prev_v) * duration);

      prev_time = curr_time;
      prev_v = v;

      // Don't store the waypoint if the
      // length of the segment is not larger thatn delta yet
      if (segment_length < delta &&
          i != (log.sample_times().size() - 1)) {
        continue;
      }

      auto x = log.data()(i * STATE_NUM);
      auto y = log.data()(i * STATE_NUM + 1);
      auto t = log.data()(i * STATE_NUM + 2);
      auto w = log.data()(i * STATE_NUM + 4);
      auto a = log.data()(i * STATE_NUM + 5);
      auto r = log.data()(i * STATE_NUM + 6);

      double state_time = log.sample_times()[i] + init_state.get_time();
      StateValue waypoint_state = {x, y, t, v, w, a, r};
      State waypoint(waypoint_state, state_time);

      trajectory.push_back(waypoint);

      total_length += segment_length;

      // Return the trajectoroy once the
      // total length is larger than epsilon
      if (total_length > epsilon) {
        break;
      }
      segment_length = 0.0;
    }

    return trajectory;
  }

}  // namespace robot_dynamic

