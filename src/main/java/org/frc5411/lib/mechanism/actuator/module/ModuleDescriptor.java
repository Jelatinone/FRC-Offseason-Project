//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.actuator.module;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.control.Controller;
import org.frc5411.lib.mechanism.Descriptor;

import lombok.Builder;
import lombok.NonNull;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N1;
//----------------------------------------------------------------------[Declaration]----------------------------------------------------------------------//
/**
 * <h1>Descriptor</h1>
 * c
 * <p>
 * 
 * @author Cody Washington
 */
@Builder(toBuilder = (true), setterPrefix = ("set"))
public class ModuleDescriptor<@NonNull Placement extends Enum<?>> implements Descriptor<Module<Placement>> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  final Double TRANSLATIONAL_GEAR_RATIO;
  final Double TRANSLATIONAL_ENCODER_OFFSET_RADIANS;
  final Double TRANSLATIONAL_MAXIMUM_VELOCITY;
  final Boolean TRANSLATIONAL_INVERTED;
  final Controller<N2,N1,N1> TRANSLATIONAL_FEEDBACK;

  final Double ROTATIONAL_GEAR_RATIO;
  final Double ROTATIONAL_ENCODER_OFFSET_RADIANS;
  final Double ROTATIONAL_MAXIMUM_VELOCITY;
  final Boolean ROTATIONAL_INVERTED;
  final Controller<N2,N1,N1> ROTATIONAL_FEEDBACK;

  final Double WHEEL_RADIUS_METERS;
  final Double WHEEL_CIRCUMFERENCE_METERS;
  final Placement WHEEL_PLACEMENT;

  @Override
  public ModuleDescriptor<Placement> clone() {
    return new ModuleDescriptor<Placement>(
      TRANSLATIONAL_GEAR_RATIO, 
      TRANSLATIONAL_ENCODER_OFFSET_RADIANS, 
      TRANSLATIONAL_MAXIMUM_VELOCITY, 
      TRANSLATIONAL_INVERTED, 
      TRANSLATIONAL_FEEDBACK, 
      ROTATIONAL_GEAR_RATIO, 
      ROTATIONAL_ENCODER_OFFSET_RADIANS, 
      ROTATIONAL_MAXIMUM_VELOCITY, 
      ROTATIONAL_INVERTED, 
      ROTATIONAL_FEEDBACK, 
      WHEEL_RADIUS_METERS, 
      WHEEL_CIRCUMFERENCE_METERS, 
      WHEEL_PLACEMENT
    );
  }
}