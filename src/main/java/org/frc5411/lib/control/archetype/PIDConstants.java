//--------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.control.archetype;
//-------------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.FieldDefaults;
//------------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Component</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */

 @Builder(toBuilder = (true), setterPrefix = ("set"))
 @Getter
 @FieldDefaults(level = AccessLevel.PUBLIC, makeFinal = (true))
public class PIDConstants {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  public final Double ProportionalGain;

  public final Double IntegralGain;

  public final Double DerivativeGain;
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Transforms the relevant PID Constants stored within this object into a 'tuned' controller object
   * @return PID controller object from stored constants
   */
  public PIDController toController() {
    return new PIDController(this);
  }
  
}
