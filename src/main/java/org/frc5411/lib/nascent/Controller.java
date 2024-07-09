//------------------------------------------------------------------------[License]----------------------------------------------------------------------------//
// Copyright 2024 Cody Washington
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//------------------------------------------------------------------------[Package]----------------------------------------------------------------------------//
package org.frc5411.lib.nascent;
//-------------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//
import org.frc5411.lib.pattern.Component;
import org.frc5411.lib.utility.Vector;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N2;

import lombok.NonNull;
//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 * <h1>Controller</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public interface Controller<@NonNull States extends Num, @NonNull Inputs extends Num, @NonNull Outputs extends Num> extends Component<Vector<Double,Inputs>> {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Corrects the state-matrix observer given the correct observed actual state-matrix, {@code Y}
   * @param Outputs Plant observed state-matrix the actuators are actually at
   */
  default void correct(final edu.wpi.first.math.Vector<Outputs> Outputs) {}

  /**
   * Enables continuous input by considering the minimum and maximum bounds to be the same point, allowing the shortest route
   * between them to be calculated
   * @param Bounds a vector containing two elements, the first is the lower bound, and the second the upper bound
   * @see #discontinuous()
   */
  default void continuous(final edu.wpi.first.math.Vector<N2> Bounds) {}

  /**
   * Disables continuous input, which allows for the shortest route between two points, a minimum and maximum bound, to be found by
   * considering them the same point.
   * @see #continuous(edu.wpi.first.math.Vector) 
   */
  default void discontinuous() {}

  /**
   * Shape-unsafe calculation of the controller's next control output, {@code U}, based on the mode of calculation and the given
   * reference state matrix, or 'set point', {@code R}, and the current system states, {@code Y}.
   * @param Reference Resized reference consisting of the allowed controller variables
   * @return Calculated controller output, {@code  U}, of any dimensions.
   */
  edu.wpi.first.math.Vector<Inputs> calculate(final edu.wpi.first.math.Vector<States> Reference);
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  /**
   * Provides a matrix (vector) of the device's 'set point', or reference states
   * @return Matrix of system states
   */
  edu.wpi.first.math.Vector<States> getStates();

  /**
   * Provides a matrix (vector) of the device's most-recent control cycle outputs
   * @return Matrix of system Outputs
   */
  edu.wpi.first.math.Vector<Inputs> getInputs();

  /**
   * Provides a matrix (vector) of the outputs of device's most-recent plant cycle outputs and the current reference.
   * @return Matrix of system outputs
   */
  edu.wpi.first.math.Vector<Outputs> getOutputs();

  /**
   * Provides a matrix (vector) of the error of device's most-recent plant cycle outputs and the current reference.
   * @return Matrix of differences between plant output(s) and control setpoint(s)
   */
  edu.wpi.first.math.Vector<Outputs> getError();


}
