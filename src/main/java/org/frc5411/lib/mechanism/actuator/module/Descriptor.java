//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.actuator.module;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//

//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Descriptor</h1>
 * c
 * <p>
 * 
 * @author Cody Washington
 */
public class Descriptor<Placement extends Enum<?>> {

  public Double TRANSLATIONAL_GEAR_RATIO;
  public Double TRANSLATIONAL_ENCODER_OFFSET_RADIANS;
  public Double TRANSLATIONAL_MAXIMUM_VELOCITY;
  public Boolean TRANSLATIONAL_INVERTED;

  public Double ROTATIONAL_GEAR_RATIO;
  public Double ROTATIONAL_ENCODER_OFFSET_RADIANS;
  public Double ROTATIONAL_MAXIMUM_VELOCITY;
  public Boolean ROTATIONAL_INVERTED;

  public Double WHEEL_RADIUS_METERS;
  public Double WHEEL_CIRCUMFERENCE_METERS;
  public Placement WHEEL_PLACEMENT;

}