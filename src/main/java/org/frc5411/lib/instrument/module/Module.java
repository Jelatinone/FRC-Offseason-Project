//------------------------------------------------------------------------[License]----------------------------------------------------------------------------//
// Copyright 2024 Cody Washington
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//------------------------------------------------------------------------[Package]----------------------------------------------------------------------------//
package org.frc5411.lib.instrument.module;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.pattern.actuator.Actuator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.Logger;

import java.util.Objects;
import java.util.Optional;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;

import static org.frc5411.lib.utility.Figure.*;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Module</h1>
 * 
 * <p>Describes an abstract Swerve Module, i.e. any device that has one, a translational actuator and two, a rotational actuator, and can be commanded to a
 * given reference of type {@link SwerveModuleState state}, and {@link Report report} back a {@link SwerveModulePosition position} from measured values. 
 * 
 * @author Cody Washington
 */
@FieldDefaults(makeFinal = (true), level = AccessLevel.PRIVATE)
public abstract class Module<@NonNull Controller, @NonNull Encoder> implements Actuator<SwerveModuleState, SwerveModulePosition> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Descriptor<Controller,Encoder> DESCRIPTION;
  ReportAutoLogged STATUS;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Module Constructor.
   * @param Description Real-world description of the system, contains relevant constants to the operation of the module
   */
  protected Module(final Descriptor<Controller,Encoder> Description) {
    DESCRIPTION = Objects.requireNonNull(Description);
    STATUS = new ReportAutoLogged();
    synchronized(STATUS) {
      STATUS.setDemand(new SwerveModuleState());
      STATUS.setEffort(new SwerveModuleState());
      STATUS.setMeasurements(new SwerveModulePosition[0]);
    }
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public final Module<Controller,Encoder> clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException();
  }

  @Override
  public synchronized void set(final SwerveModuleState Demand) {
    STATUS.setDemand(Demand);
  }

  @Override
  public synchronized void periodic() {
    synchronized(STATUS) {
      update(STATUS);
      final var Reference = getState();
      final var Effort = new SwerveModuleState();
      if(getConnection() || Reference != (null)) {
          setTranslationalVoltage(Effort.speedMetersPerSecond = unwrap(
            DESCRIPTION.TranslationalFeedback.calculate(wrap(
              STATUS.getTranslationalVelocity(), 
              Reference.speedMetersPerSecond * Math.cos(
                unwrap(DESCRIPTION.RotationalFeedback.getError())) / DESCRIPTION.Radius
            ))
          ));
          final var Position = getRotationalPosition();
          if(Reference.angle != (null) && Position.isPresent()) {
            setRotationalVoltage((
              Effort.angle = Rotation2d.fromRotations(unwrap(
              DESCRIPTION.RotationalFeedback.calculate(wrap(
                Position.get().getRadians(), 
                Reference.angle.getRadians()))))
            ).getRotations());
          }          
      } else {
        cease();
      }
      STATUS.setEffort(Effort);
    }
    Logger.processInputs(
      getIdentity(),STATUS);   
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
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  @Override
  public Report getReport() {
    return STATUS;
  }

  /**
   * Provides the current position (angular displacement) of the module's rotational axis with an offset, interpreted from the current measurement of the system recorded
   * within the {@link #update(org.frc5411.lib.pattern.Report)}
   * @return Position of the rotational controller's axis of rotation in radians as a Rotation2d Object
   */
  public Optional<Rotation2d> getRotationalPosition() {
    return getMeasurement().map(Measurement -> Measurement.angle.minus(DESCRIPTION.RotationalOffset));
  }

  /**
   * Provides the current position (translational displacement) of the module's translational axis with an offset, interpreted from the current measurement of the system
   * recorded within the {@link #update(org.frc5411.lib.pattern.Report)}
   * @return Position of the translational controller's axis of rotation in meters as a Double Object
   */
  public Optional<Double> getTranslationPosition() {
    return getMeasurement().map(Measurement -> Measurement.distanceMeters - DESCRIPTION.TranslationalOffset);
  }

  
  @Override
  public Descriptor<Controller,Encoder> getDescriptor() {
    return DESCRIPTION;
  }

  @Override
  public String getIdentity() {
    return String.format(
      ("Module-[%s]"), 
      DESCRIPTION.Identity.name());
  }
}
