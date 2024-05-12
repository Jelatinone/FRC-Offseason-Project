//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.pattern.actuator.module;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.pattern.actuator.Actuator;

import lombok.NonNull;

import java.util.Objects;
import java.lang.CloneNotSupportedException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Module</h1>
 * 
 * <p>Describes an abstract Swerve Module, i.e. any device that has one, a translational actuator and two, a rotational actuator, and can be commanded to a
 * given reference of type {@link SwerveModuleState state}, and @{link Report report} back a {@link SwerveModulePosition position} from measured values. 
 * 
 * @author Cody Washington
 */
public abstract class Module<@NonNull Placement extends Enum<?>> implements Actuator<SwerveModuleState, SwerveModulePosition> {
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
  //------------------------------------------------------------------------[Methods]-------------------------------------------------------------------------//
  @Override
  public Module<Placement> clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException();
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
    return Units.rotationsPerMinuteToRadiansPerSecond(
      ((Report) getReport()).getTranslationalVelocity());
  }

  /**
   * Provides the current speed of the module, interpreted from the angular speed, omega.
   * @return Velocity of the rotational controller's axis of rotation in meters/second
   */
  public Double getRotationalVelocity() {
    return Units.rotationsPerMinuteToRadiansPerSecond(
      ((Report) getReport()).getRotationalVelocity());
  }

  /**
   * Provides the current position (angular displacement) of the module's rotational axis with an offset, interpreted from the current measurement of the system recorded
   * within the {@link #update(org.frc5411.lib.mechanism.Report)}
   * @return Position of the rotational controller's axis of rotation in radians as a Rotation2d Object
   */
  public Rotation2d getRotationalPosition() {
    return getMeasurement().angle.minus(getRotationalOffset());
  }

  /**
   * Provides the current position (translational displacement) of the module's translational axis with an offset, interpreted from the current measurement of the system
   * recorded within the {@link #update(org.frc5411.lib.mechanism.Report)}
   * @return Position of the translational controller's axis of rotation in meters as a Double Object
   */
  public Double getTranslationPosition() {
    return getMeasurement().distanceMeters - getTranslationalOffset();
  }

  /**
   * Provides the constant rotational angular displacement offset of the rotational controller's offset of the encoder feedback
   * @return Positional offset of the rotational controller
   */
  public Rotation2d getRotationalOffset() {
    return Rotation2d.fromRadians(DESCRIPTION.RotationalOffset);
  }

  /**
   * Provides the constant translational angular displacement offset of the translational controller's offset of the encoder feedback
   * @return Positional offset of the translational controller
   */
  public Double getTranslationalOffset() {
    return DESCRIPTION.TranslationalOffset; 
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
    return DESCRIPTION.Placement;
  }

}
