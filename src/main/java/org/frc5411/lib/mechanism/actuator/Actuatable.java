//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.actuator;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.mechanism.Component;

import edu.wpi.first.util.struct.StructSerializable;
import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Actuatable</h1>
 * 
 * <p>Describes any {@link Component} that can also be actuated, like a motor or pneumatic device. This is essentially any device that can be
 * actuated, i.e. controlling the Effort of the controller. This provides base implementation for essentially any system that can be moved 
 * (but not limited to) to a given Demand, such as a Drivebase, Piston, or Module.
 * 
 * @author Cody Washington
 */
public interface Actuatable<@NonNull Reference, @NonNull Measurement extends StructSerializable> extends Component<Measurement> {
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
