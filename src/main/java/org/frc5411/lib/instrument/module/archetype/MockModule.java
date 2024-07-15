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
package org.frc5411.lib.instrument.module.archetype;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.instrument.module.Descriptor;
import org.frc5411.lib.instrument.module.Module;
import org.frc5411.lib.instrument.module.Report;
import org.frc5411.lib.nouveau.StandardRegister;
import org.frc5411.lib.utility.Aggregator;
import org.frc5411.lib.utility.Figures;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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
   * Mock Module Constructor.
   * @param Descriptor Real-world {@link #getDescriptor() descriptor} of the system, contains relevant constants to the operation of the module
   */
  public MockModule(final Descriptor<DCMotorSim,Optional<Object>> Descriptor) {
    super(Descriptor);

    TRANSLATIONAL_POSITIONS = StandardRegister
      .getInstance()
      .register(() -> Optional.of(
          Descriptor.TranslationalController.getAngularPositionRad()
        )
      );
    ROTATIONAL_POSITIONS = StandardRegister
      .getInstance()
      .register(() -> Optional.of(
          Descriptor.RotationalController.getAngularPositionRad()
        )
      );
        
    UPDATE_TIMESTAMPS = StandardRegister
      .getInstance()
      .timestamp();

    DISCRETE_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1E6D, 
      (Previous, Current) -> Current - Previous);
    configure();
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized void cease() {
    getDescriptor().TranslationalController
      .setInputVoltage((0D));
    getDescriptor().RotationalController
      .setInputVoltage((0D));
  }

  @Override
  public synchronized void configure() {
    cease();
    synchronized(this) {
      getDescriptor().TranslationalController.setState(
        Units.rotationsToRadians(Math.random()), 
        (0D));
      getDescriptor().RotationalController.setState(
        Units.rotationsToRadians(Math.random()), 
        (0D));
      getDescriptor().RotationalFeedback.continuous(
        VecBuilder.fill(
          -Figures.PI, 
          +Figures.PI
      ));
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

    DISCRETE_AGGREGATOR.aggregate();

    getDescriptor().TranslationalController.update(DISCRETE_AGGREGATOR.getAggregated());
    getDescriptor().RotationalController.update(DISCRETE_AGGREGATOR.getAggregated());

    synchronized(Article) {
      Article.setTranslationalVoltage(getInput().orElseThrow().speedMetersPerSecond);
      Article.setTranslationalAmperage(Math.abs(getDescriptor().TranslationalController.getCurrentDrawAmps()));
      Article.setTranslationalVelocity(getDescriptor().TranslationalController.getAngularVelocityRadPerSec() * getDescriptor().Radius);
      Article.setTranslationalConnected((true));

      Article.setRotationalVoltage(getInput().orElseThrow().angle.getRotations());
      Article.setRotationalAmperage(Math.abs(getDescriptor().RotationalController.getCurrentDrawAmps()));
      Article.setRotationalVelocity(getDescriptor().RotationalController.getAngularVelocityRadPerSec());
      Article.setRotationalConnected((true));   

      Article.setConnected(Article.isTranslationalConnected() && Article.isRotationalConnected());
      Article.setOutput(
        new SwerveModuleState(
          Article
            .getTranslationalVelocity(),
          Rotation2d
            .fromRotations(getDescriptor().RotationalController.getAngularPositionRotations())
            .minus(getDescriptor().RotationalOffset)
      ));

      final double[] Translations, Rotations;
      synchronized(TRANSLATIONAL_POSITIONS) {
        Translations = TRANSLATIONAL_POSITIONS
          .stream()
          .mapToDouble((Position) -> 
            Position.orElseThrow().doubleValue())
          .toArray();
        TRANSLATIONAL_POSITIONS.clear();
      }
      synchronized(ROTATIONAL_POSITIONS) {
        Rotations = ROTATIONAL_POSITIONS
          .stream()
          .mapToDouble((Position) -> 
            Position.orElseThrow().doubleValue())
          .toArray();
        ROTATIONAL_POSITIONS.clear();
      }
      synchronized(UPDATE_TIMESTAMPS) {
        Article.setTimestamps(UPDATE_TIMESTAMPS
          .stream()
          .mapToDouble(Number::doubleValue)
          .toArray());
        UPDATE_TIMESTAMPS.clear();
      }

      Article.setMeasurements(IntStream.range((0), Figures.minimum(Translations.length, Rotations.length)).mapToObj((Index) -> 
        new SwerveModulePosition(
          (Translations[Index] - getDescriptor().TranslationalOffset) * getDescriptor().Radius, 
          Rotation2d
            .fromRadians(Rotations[Index])
            .minus(getDescriptor().RotationalOffset))
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