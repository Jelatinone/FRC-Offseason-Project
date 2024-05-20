//--------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.control;
//-------------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

import lombok.NonNull;
//------------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Component</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public interface Controller<@NonNull States extends Num, @NonNull Inputs extends Num, @NonNull Outputs extends Num> {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
    /**
   * Resets this controller to a given initial position and all other relevant data.
   */
  default void reset() {}

  /**
   * Corrects the state-matrix observer given the correct observed actual state-matrix, {@code Y}
   * @param Outputs Plant observed state-matrix the actuators are actually at
   */
  default void correct(final Matrix<Outputs,N1> Outputs) {}

  /**
   * Shape-unsafe calculation of the controller's next control output, {@code U}, based on the mode of calculation and the given
   * reference state matrix, or 'set point', {@code R}, and the current system states, {@code Y}.
   * @param Reference Resized reference consisting of the allowed controller variables
   * @return Calculated controller output, {@code  U}, of any dimensions.
   */
  Matrix<Inputs,N1> calculate(final Matrix<States,N1> Reference);
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//

  /**
   * Provides a matrix (vector) of the device's 'set point', or reference states
   * @return Matrix of system states
   */
  Matrix<States,N1> getStates();

  /**
   * Provides a matrix (vector) of the device's most-recent control cycle outputs
   * @return Matrix of system Outputs
   */
  Matrix<Inputs,N1> getInputs();

  /**
   * Provides a matrix (vector) of the error of device's most-recent plant cycle outputs and the current reference.
   * @return Matrix of system inputs
   */
  Matrix<Outputs,N1> getError();
}
