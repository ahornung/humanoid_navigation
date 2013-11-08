/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FOOTSTEP_PLANNER_FOOTSTEP_H_
#define FOOTSTEP_PLANNER_FOOTSTEP_H_

#include <footstep_planner/PlanningState.h>


namespace footstep_planner
{
/**
 * @brief A class representing a footstep (i.e. a translation and rotation
 * of a specific foot with respect to the supporting leg) that can be
 * performed by a humanoid robot.
 *
 * Since the underlying SBPL is working on discretized states the footsteps
 * are also a discretized translations and rotations.
 */
class Footstep
{
public:
  /**
   * @brief The constructor takes the continuous translation and rotation
   * of the footstep and calculates the respective discretized footstep
   * based on the parameters of the discretization.
   *
   * @param x The (continuous) translation in x direction.
   * @param y The (continuous) translation in y direction.
   * @param theta The (continuous) rotation.
   * @param cell_size Parameter to discretize the translation (see
   * PlanningState for further explanation).
   * @param num_angle_bins Parameter to discretize the rotation (see
   * PlanningState for further explanation).
   * @param max_hash_size The maximal hash size.
   */
  Footstep(double x, double y, double theta,
           double cell_size, int num_angle_bins, int max_hash_size);
  ~Footstep();

  /**
   * @brief Performs this footstep (translation and rotation) on a given
   * planning state.
   *
   * @param current The planning state representing the robot's current
   * supporting leg.
   * @return The resulting planning state.
   */
  PlanningState performMeOnThisState(const PlanningState& current) const;

  /**
   * @brief Reverse this footstep on a given planning state.
   *
   * @param current The planning state representing the robot's current
   * supporting leg.
   * @return The reversed planning state, i.e. the state the robot was in
   * if this footstep had not been performed.
   */
  PlanningState reverseMeOnThisState(const PlanningState& current) const;

private:
  /// Typedef representing the (discretized) translation of the footstep.
  typedef std::pair<int, int> footstep_xy;

  /// Initialization method called within the constructor.
  void init(double x, double y);

  /**
   * @brief Discretizes the translation of the footstep for a certain
   * (discretized) orientation of a possible state.
   *
   * @param leg The supporting leg of the possible state.
   * @param global_theta The (discretized) orientation of the possible
   * state.
   * @param x The (continuous) translation in x direction.
   * @param y The (continuous) translation in y direction.
   * @param footstep_x The resulting (discretized) translation in x
   * direction.
   * @param footstep_y The resulting (discretized) translation in y
   * direction.
   * @return The (discretized) orientation of the resulting state after
   * performing the footstep. This is used to calculate the (discretized)
   * reversed footstep.
   */
  int calculateForwardStep(Leg leg, int global_theta,
                           double x, double y,
                           int* footstep_x, int* footstep_y) const;

  /// The (discretized) rotation of the footstep.
  int ivTheta;

  /// The parameter for the discretization of the translation.
  double ivCellSize;
  /// The parameter for the discretization of the rotation.
  int ivNumAngleBins;

  /// The maximal hash size.
  int ivMaxHashSize;

  /// The (discretized) translation(s) for a left supporting foot.
  std::vector<footstep_xy> ivDiscSuccessorLeft;
  /// The (discretized) translation(s) for a right supporting foot.
  std::vector<footstep_xy> ivDiscSuccessorRight;
  /// The reversed (discretized) translation(s) for a left supporting foot.
  std::vector<footstep_xy> ivDiscPredecessorLeft;
  /// The reversed (discretized) translation(s) for a right supporting foot.
  std::vector<footstep_xy> ivDiscPredecessorRight;
};
} // end of namespace

#endif  // FOOTSTEP_PLANNER_FOOTSTEP_H_
