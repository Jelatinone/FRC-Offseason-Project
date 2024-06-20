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
import org.frc5411.lib.control.archetype.PIDController;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.nouveau.StandardRegister;
import org.frc5411.lib.utility.Aggregator;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.Optional;
import java.util.Queue;
import java.util.stream.IntStream;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>MockModule</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(makeFinal = (true), level = AccessLevel.PRIVATE)
public class MockModule extends Module<DCMotorSim,Optional<Object>> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Queue<Optional<Number>> TRANSLATIONAL_POSITIONS;
  Queue<Optional<Number>> ROTATIONAL_POSITIONS;

  Queue<Double> UPDATE_TIMESTAMPS;

  Aggregator<Double> DISCRETE_AGGREGATOR;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Simulated Module Constructor.
   * @param Descriptor Real-world getDescriptor() of the system, contains relevant constants to the operation of the module
   */
  public MockModule(final Descriptor<DCMotorSim,Optional<Object>> Descriptor) {
    super(Descriptor);

    TRANSLATIONAL_POSITIONS = StandardRegister
      .getInstance()
      .register(() -> Optional.of(
        Descriptor.TranslationalController.getAngularPositionRotations()));
    ROTATIONAL_POSITIONS = StandardRegister
      .getInstance()
      .register(() -> Optional.of(
        Descriptor.RotationalController.getAngularPositionRad()));
        
    UPDATE_TIMESTAMPS = StandardRegister
      .getInstance()
      .timestamp();

    DISCRETE_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1e6, 
      (Previous, Current) -> Current - Previous);
    configure();
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized void cease() {
    getDescriptor().TranslationalController.setState((0D), (0D));
    getDescriptor().RotationalController.setState((0D), (0D));
  }

  @Override
  public synchronized void configure() {
    cease();
    synchronized(this) {
      ((PIDController) getDescriptor().RotationalFeedback).enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  @Override
  public synchronized void close() {
    TRANSLATIONAL_POSITIONS.clear();
    ROTATIONAL_POSITIONS.clear();
    UPDATE_TIMESTAMPS.clear();
  }

  @Override
  public synchronized void update(final org.frc5411.lib.pattern.Report<@NonNull SwerveModulePosition> Record) {
    final var Article = (Report) Record;

    getDescriptor().TranslationalController.update(DISCRETE_AGGREGATOR.aggregate());
    getDescriptor().RotationalController.update(DISCRETE_AGGREGATOR.getAggregated());

    synchronized(Article) {
      Article.setTranslationalAmperage(getDescriptor().TranslationalController.getCurrentDrawAmps());
      Article.setTranslationalVelocity(getDescriptor().TranslationalController.getAngularVelocityRPM() / getDescriptor().TranslationalReduction);
      Article.setTranslationalConnected((true));

      Article.setRotationalAmperage(getDescriptor().RotationalController.getCurrentDrawAmps());
      Article.setRotationalVelocity(getDescriptor().RotationalController.getAngularVelocityRPM() / getDescriptor().RotationalReduction);
      Article.setRotationalConnected((true));   

      Article.setConnected(Article.isTranslationalConnected() && Article.isRotationalConnected());

      final double[] Translations, Rotations;
      synchronized(TRANSLATIONAL_POSITIONS) {
        Translations = TRANSLATIONAL_POSITIONS
          .stream()
          .mapToDouble((Position) -> 
            Position.get().doubleValue() / getDescriptor().TranslationalReduction * getDescriptor().Radius)
          .toArray();
        TRANSLATIONAL_POSITIONS.clear();
      }
      synchronized(ROTATIONAL_POSITIONS) {
        Rotations = ROTATIONAL_POSITIONS
          .stream()
          .mapToDouble((Position) -> 
            Position.get().doubleValue() / getDescriptor().RotationalReduction)
            .toArray();
        ROTATIONAL_POSITIONS.clear();
      }
      synchronized(UPDATE_TIMESTAMPS) {
        Article.setTimestamps(UPDATE_TIMESTAMPS
          .stream()
          .mapToDouble(Double::doubleValue)
          .toArray());
        UPDATE_TIMESTAMPS.clear();
      }        
      Article.setMeasurements(IntStream.range((0), Math.min(Translations.length, Rotations.length)).mapToObj((Index) -> 
        new SwerveModulePosition(
          Translations[Index], 
          Rotation2d.fromRadians(Rotations[Index]))
      ).toArray(SwerveModulePosition[]::new));
    }
  }
  //-----------------------------------------------------------------------[Mutators]--------------------------------------------------------------------------//
  /**
   * Mutates the current voltage applied to the module's translational motor controller
   * @param Demand Voltage sent to the controller object 
   */
  protected void setTranslationalVoltage(final double Demand) {
    getDescriptor().TranslationalController.setInputVoltage(MathUtil.clamp(Demand, (-12D), (12D)));
  }

  /**
   * Mutates the current voltage applied to the module's rotational motor controller
   * @param Demand Voltage sent to the controller object 
   */
  protected void setRotationalVoltage(final double Demand) {
    getDescriptor().RotationalController.setInputVoltage(MathUtil.clamp(Demand, (-12D), (12D)));
  }
}