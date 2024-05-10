//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.observer.gyroscope;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import lombok.Builder;

import org.frc5411.lib.mechanism.Descriptor;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Descriptor</h1>
 * c
 * <p>
 * 
 * @author Cody Washington
 */
@Builder(toBuilder = (true), setterPrefix = ("set"))
public class GyroscopeDescriptor implements Descriptor<Gyroscope> {

  final Double YAW_ENCODER_OFFSET_RADIANS;
  final Double PITCH_ENCODER_OFFSET_RADIANS;
  final Double ROLL_ENCODER_OFFSET_RADIANS;

  @Override
  public GyroscopeDescriptor clone() {
    return new GyroscopeDescriptor(
      YAW_ENCODER_OFFSET_RADIANS, 
      PITCH_ENCODER_OFFSET_RADIANS, 
      ROLL_ENCODER_OFFSET_RADIANS);
  }

}