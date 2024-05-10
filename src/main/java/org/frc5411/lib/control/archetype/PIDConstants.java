//--------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.control.archetype;
//-------------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//

import lombok.Builder;

//------------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Component</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */

 @Builder(toBuilder = (true), setterPrefix = ("set"))
public class PIDConstants {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  public final Double PROPORTIONAL_GAIN;

  public final Double INTEGRAL_GAIN;

  public final Double DERIVATIVE_GAIN;
  
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Transforms the relevant PID Constants stored within this object into a 'tuned' controller object
   * @return PID controller object from stored constants
   */
  public PIDController toController() {
    return new PIDController(this);
  }
  
}
