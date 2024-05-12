//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.pattern.actuator.module;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import lombok.Builder;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
import lombok.experimental.FieldNameConstants;
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
@FieldNameConstants(asEnum = true)
public class Descriptor<@NonNull Placement extends Enum<?>> implements org.frc5411.lib.pattern.Descriptor<Module<Placement>> {
  
  Double TranslationalReduction;
  Double TranslationalOffset;
  Double TranslationalVelocity;
  Boolean TranslationalInverted;
  Double TranslationalAcceleration;

  Double RotationalReduction;
  Double RotationalOffset;
  Double RotationalVelocity;
  Boolean RotationalInverted;

  Double Radius;
  Double Circumference;
  Placement Placement;

  @Override
  public Descriptor<Placement> clone() {
    return new Descriptor<Placement>(
      TranslationalReduction, 
      TranslationalOffset, 
      TranslationalVelocity, 
      TranslationalInverted, 
      TranslationalAcceleration, 
      RotationalReduction, 
      RotationalOffset, 
      RotationalVelocity, 
      RotationalInverted, 
      Radius, 
      Circumference, 
      Placement);
  }

}
