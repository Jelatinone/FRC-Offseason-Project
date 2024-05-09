//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.actuator;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.mechanism.Component;
import org.frc5411.lib.mechanism.Report;

import edu.wpi.first.util.WPISerializable;

import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Actuatable</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public interface Actuatable<@NonNull Reference, @NonNull Measurement extends WPISerializable, @NonNull Loggable extends Report<Measurement>> extends Component<Measurement, Loggable> {
  //------------------------------------------------------------------------[Methods]-------------------------------------------------------------------------//
  /**
   * Mutates the current demand state of the actuator to a different state, but does not immediately process the correct actuator effort required
   * to reach this demand state, instead this is done during {@link #periodic()}.
   * @param Demand Reference state to reach
   */
  void set(final Reference Demand);

  /**
   * Immediately stops this actuator in such a way that {@link #set(Object)} can be called again after.
   */
  void cease();

  /**
   * Provides the current reference state, in other words the Demand set by calling {@link #set(Object)}.
   * @return Struct of current state
   */
  Reference getState();

  /**
   * Provides the current controller input to the actuator, in other words the controller effort updated internally each {@link #periodic()} call.
   * @return Struct of controller effort
   */
  Reference getInput();  
}
