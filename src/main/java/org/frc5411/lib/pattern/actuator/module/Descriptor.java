//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.pattern.actuator.module;
import org.frc5411.lib.control.Controller;
import org.frc5411.lib.pattern.Component;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import lombok.Builder;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]----------------------------------------------------------------------//
/**
 * <h1>Descriptor</h1>
 * 
 * @see org.frc5411.lib.pattern.Descriptor Descriptor
 * 
 * @author Cody Washington
 */
@Builder(toBuilder = true, setterPrefix = ("set"))
@FieldDefaults(makeFinal = true)
public class Descriptor<@NonNull Placement extends Enum<?>> extends org.frc5411.lib.pattern.Descriptor<Component<SwerveModulePosition>> {
  
  Double TranslationalReduction;
  Double TranslationalOffset;
  Double TranslationalVelocity;
  Double TranslationalAcceleration;
  Boolean TranslationalInverted;
  Controller<N2,N1,N1> TranslationalFeedback;
  
  Double RotationalReduction;
  Double RotationalOffset;
  Double RotationalVelocity;
  Boolean RotationalInverted;
  Controller<N2,N1,N1> RotationalFeedback;

  Double Radius;
  Double Circumference;
  Placement Placement;

  @Override
  public Descriptor<Placement> clone() {
    return new Descriptor<Placement>(
      TranslationalReduction, 
      TranslationalOffset, 
      TranslationalVelocity, 
      TranslationalAcceleration, 
      TranslationalInverted,
      TranslationalFeedback, 
      RotationalReduction, 
      RotationalOffset, 
      RotationalVelocity, 
      RotationalInverted, 
      RotationalFeedback,
      Radius, 
      Circumference, 
      Placement);
  }
}
