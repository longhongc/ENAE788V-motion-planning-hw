/* robot.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <array>
#include <vector>
#include <utility>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/continuous_state.h>

// The total number of state
#define STATE_NUM 7

using Coord2D = std::pair<double, double>;

// The robot state
// x, y, theta, v, w, a, r
using StateValue = std::array<double, STATE_NUM>;

/**
 * @Brief  Data structure for robot state
 */
class State {
  public:
    State(StateValue& value, double time = 0);

    /**
     * @Brief  Set the value of state
     *
     * @Param value
     */
    void set_value(StateValue& value);

    /**
     * @Brief  Get the value of state
     *
     * @Returns
     */
    StateValue get_value();

    /**
     * @Brief  Get the 2D coordinate of the state
     *
     * @Returns
     */
    Coord2D get_coord();

    /**
     * @Brief  Get the time of the state
     *
     * @Returns
     */
    double get_time();

  private:
    /**
     * @Brief  The robot state
     *         x, y, theta, velocity, angular velocity,
     *         accleration, angular acceleration
     */
    StateValue value_;

    /**
     * @Brief  The time of this state
     */
    double time_;
};

/**
 * @Brief  Contains information and function for robot dynamic
 */
namespace robot_dynamic {
  /**
   * @Brief  Maximum linear velocity
   */
  const double V_MAX = 5;

  /**
   * @Brief  Minimum linear velocity
   *         The minimum of the system is actually -5
   *         Use small positive number to constraint the robot
   *         to only go forward and smoother path
   */
  const double V_MIN = 1;

  /**
   * @Brief  Maximum angular velocity
   *         The maximum of the system is actually 0.5 * M_PI
   *         Use smaller magnitude for smoother path
   */
  const double W_MAX = 0.25 * M_PI;

  /**
   * @Brief  Minimum angular velocity
   *         The minimum of the system is actually -0.5 * M_PI
   *         Use smaller magnitude for smoother path
   */
  const double W_MIN = -0.25 * M_PI;

  /**
   * @Brief  Maximum linear acceleration
   */
  const double ACC_MAX = 2;

  /**
   * @Brief  Minimum linear acceleration
   */
  const double ACC_MIN = -2;

  /**
   * @Brief  Maximum angular acceleration
   */
  const double GAMMA_MAX = 0.5 * M_PI;

  /**
   * @Brief  Minimum angular acceleration
   */
  const double GAMMA_MIN = -0.5 * M_PI;


  /**
   * @Brief  The Half Car system constructed using drake
   */
  class HalfCarSystem : public drake::systems::LeafSystem<double> {
    public:
      HalfCarSystem();

    private:
      /**
       * @Brief  The system dynamic
       *
       * @Param context  The state
       * @Param derivatives The derivatives of the state
       */
      void DoCalcTimeDerivatives(
          const drake::systems::Context<double>& context,
          drake::systems::ContinuousState<double>* derivatives) const;

      /**
       * @Brief  The output of the system
       *
       * @Param context The state
       * @Param output  The output which is the state in this system
       */
      void CopyStateOut(
          const drake::systems::Context<double>& context,
          drake::systems::BasicVector<double>* output) const;
  };

  /**
   * @Brief  The forward simulation of the Half Car system
   *
   * @Param init_state The initial state (x, y, theta, v, w, a, r, and time)
   * @Param epsilon The epsilon in RRT which is roughly the length of each edge
   * @Param delta The resoluiton length for collision checking.
   *              There is waypoint for every delta length in the trajectory
   * @Param boundary_time The maximum simulation time
   *
   * @Returns
   */
  std::vector<State> forward_simulation(
      State& init_state,
      double epsilon = 10,
      double delta = 5,
      double boundary_time = 3);
}  // namespace robot_dynamic
