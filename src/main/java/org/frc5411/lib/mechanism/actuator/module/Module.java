//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.actuator.module;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.mechanism.actuator.Actuatable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Objects;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Module</h1>
 * c
 * <p>
 * 
 * @author Cody Washington
 */
public abstract class Module<Placement extends Enum<?>> implements Actuatable<SwerveModuleState, SwerveModulePosition, Report> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  private final Descriptor<Placement> DESCRIPTION;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Module Constructor.
   * @param Description Real-world description of the system, contains relevant constants to the operation of the module
   */
  protected Module(final Descriptor<Placement> Description) {
    DESCRIPTION = Objects.requireNonNull(Description);
  }
  //-----------------------------------------------------------------------[Mutators]--------------------------------------------------------------------------//
  /**
   * Mutates the current voltage applied to the module's translational motor controller
   * @param Demand Voltage sent to the controller object 
   */
  protected abstract void setTranslationalVoltage(final double Demand);

  /**
   * Mutates the current voltage applied to the module's rotational motor controller
   * @param Demand Voltage sent to the controller object 
   */
  protected abstract void setRotationalVoltage(final double Demand);

  /**
   * Mutates the current state of the translational controller to lock rotational movement of the axis of rotation
   * @param Locked Whether the translational axis is locked
   */
  protected abstract void setTranslationalLocked(final Boolean Locked);

  /**
   * Mutates the current state of the rotational controller to lock rotational movement of the axis of rotation
   * @param Locked Whether the rotational axis is locked
   */
  protected abstract void setRotationalLocked(final Boolean Locked);
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  /**
   * Provides the current speed of the module, interpreted from the angular speed, omega.
   * @return Velocity of the translational controller's axis of rotation in meters/second
   */
  public Double getTranslationalVelocity() {
    return ((Report) getReport()).TranslationalVelocityRotationsMinute;
  }

  /**
   * Provides the current speed of the module, interpreted from the angular speed, omega.
   * @return Velocity of the rotational controller's axis of rotation in meters/second
   */
  public Double getRotationalVelocity() {
    return ((Report) getReport()).RotationalVelocityRotationsMinute;
  }

  /**
   * Provides the current position (angular displacement) of the module's rotational axis, interpreted from the current measurement of the system recorded
   * within the {@link #update(Report)}
   * @return Position of the rotational controller's axis of rotation in radians as a Rotation2d Object
   */
  public Rotation2d getRotationalPosition() {
    return getMeasurement().angle;
  }

  /**
   * Provides the current position (translational displacement) of the module's translational axis, interpreted from the current measurement of the system
   * recorded within the {@link #update(Report)}
   * @return Position of the translational controller's axis of rotation in meters as a Double Object
   */
  public Double getTranslationPosition() {
    return getMeasurement().distanceMeters;
  }

  /**
   * Provides the constant rotational angular displacement offset of the rotational controller's offset of the encoder feedback
   * @return Positional offset of the rotational controller
   */
  public Rotation2d getRotationalOffset() {
    return Rotation2d.fromRadians(DESCRIPTION.ROTATIONAL_ENCODER_OFFSET_RADIANS);
  }

  /**
   * Provides the constant translational angular displacement offset of the translational controller's offset of the encoder feedback
   * @return Positional offset of the translational controller
   */
  public Rotation2d getTranslationalOffset() {
    return Rotation2d.fromRadians(DESCRIPTION.TRANSLATIONAL_ENCODER_OFFSET_RADIANS); 
  }

  /**
   * Provides the real-world description of the module, essentially an object makeup of the system's constants.
   * @return Description of this module
   */
  public Descriptor<Placement> getDescriptor() {
    return DESCRIPTION;
  }

  /**
   * Provides the real-world placement of the module relative to  the wheel-base, this is a non-enforced requirement of the module and has
   * no effect on the operations, but is instead used to make the modules distinct from one-another.
   * @return Placement of the module (wheel-base relative)
   */
  public Placement getPlacement() {
    return DESCRIPTION.WHEEL_PLACEMENT;
  }
} 
