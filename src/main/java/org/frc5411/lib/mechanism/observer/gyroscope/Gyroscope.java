//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.observer.gyroscope;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.mechanism.Component;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.Objects;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Gyroscope</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public abstract class Gyroscope implements Component<Rotation3d> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  private final GyroscopeDescriptor DESCRIPTION;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Module Constructor.
   * @param Description Real-world description of the system, contains relevant constants to the operation of the module
   */
  protected Gyroscope(final GyroscopeDescriptor Description) {
    DESCRIPTION = Objects.requireNonNull(Description);
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  /**
   * Provides the constant rotational angular displacement of the gyroscope's yaw axis (z-axis)
   * @return Position of the magnetic encoder of the gyroscope
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromRadians(getReport().Measurement.getZ()).minus(Rotation2d.fromRadians(DESCRIPTION.YAW_ENCODER_OFFSET_RADIANS));
  }

  /**
   * Provides the constant rotational angular displacement of the gyroscope's pitch axis (y-axis)
   * @return Position of the magnetic encoder of the gyroscope
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromRadians(getReport().Measurement.getY()).minus(Rotation2d.fromRadians(DESCRIPTION.PITCH_ENCODER_OFFSET_RADIANS));
  }

  /**
   * Provides the constant rotational angular displacement of the gyroscope's roll axis (x-axis)
   * @return Position of the magnetic encoder of the gyroscope
   */
  public Rotation2d getRoll() {
    return Rotation2d.fromRadians(getReport().Measurement.getX()).minus(Rotation2d.fromRadians(DESCRIPTION.ROLL_ENCODER_OFFSET_RADIANS));
  }

  /**
   * Provides the constant rotational angular displacement offset of the gyroscope's yaw axis (z-axis)
   * @return Positional offset of the magnetic encoder of the gyroscope
   */
  public Rotation2d getYawOffset() {
    return Rotation2d.fromRadians(DESCRIPTION.YAW_ENCODER_OFFSET_RADIANS);
  }

  /**
   * Provides the constant rotational angular displacement offset of the gyroscope's pitch axis (y-axis)
   * @return Positional offset of the magnetic encoder of the gyroscope
   */
  public Rotation2d getPitchOffset() {
    return Rotation2d.fromRadians(DESCRIPTION.PITCH_ENCODER_OFFSET_RADIANS);
  }

  /**
   * Provides the constant rotational angular displacement offset of the gyroscope's roll axis (x-axis)
   * @return Positional offset of the magnetic encoder of the gyroscope
   */
  public Rotation2d getRollOffset() {
    return Rotation2d.fromRadians(DESCRIPTION.ROLL_ENCODER_OFFSET_RADIANS);
  }

  /**
   * Provides the velocity (speed in rad/sec) of the latest measurement
   * @return Velocity of the magnetic encoder of the gyroscope
   */
  public Rotation3d getVelocity() {
    return ((GyroscopeReport) getReport()).MeasurementVelocityRotationsMinute;
  }

    /**
   * Provides the real-world description of the module, essentially an object makeup of the system's constants.
   * @return Description of this module
   */
  public GyroscopeDescriptor getDescriptor() {
    return DESCRIPTION;
  }
}
