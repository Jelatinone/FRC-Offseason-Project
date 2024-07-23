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
import org.frc5411.lib.utility.Figures;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import java.util.Optional;
import java.util.Queue;
import java.util.stream.IntStream;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>SparkModule</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(makeFinal = (true), level = AccessLevel.PRIVATE)
public class SparkModule extends Module<CANSparkBase,CANcoder> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Queue<Optional<Number>> TRANSLATIONAL_POSITIONS;
  RelativeEncoder TRANSLATIONAL_ENCODER;
  
  Queue<Optional<Number>> ROTATIONAL_POSITIONS;

  Queue<Double> UPDATE_TIMESTAMPS;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Spark Module Constructor.
   * @param Descriptor Real-world {@link #getDescriptor() descriptor} of the system, contains relevant constants to the operation of the module
   */
  public SparkModule(final Descriptor<CANSparkBase,CANcoder> Descriptor) {
    super(Descriptor);

    TRANSLATIONAL_ENCODER = getDescriptor().TranslationalController.getEncoder();
    TRANSLATIONAL_POSITIONS = StandardRegister
      .getInstance()
      .register(() -> Optional.ofNullable(
          getConnection()? 
            TRANSLATIONAL_ENCODER
              .getPosition(): 
            (null)
      ));

    ROTATIONAL_POSITIONS = StandardRegister
      .getInstance()
      .register(() -> Optional.ofNullable(
          getConnection()? 
            getDescriptor().RotationalEncoder
              .getAbsolutePosition()
              .refresh()
              .getValue(): 
            (null)
      ));
        
    UPDATE_TIMESTAMPS = StandardRegister
      .getInstance()
      .timestamp();

    configure();
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized void configure() {
    cease();
    synchronized(this) {
      getDescriptor().TranslationalController.restoreFactoryDefaults();
      getDescriptor().RotationalController.restoreFactoryDefaults();

      getDescriptor().TranslationalController.setCANTimeout((250));
      getDescriptor().RotationalController.setCANTimeout((250));

      //Knowledge from beyond...?
      for(int Iterations = (0); Iterations < (30); Iterations++) {
        getDescriptor().TranslationalController.setInverted(getDescriptor().TranslationalInverted);
        getDescriptor().RotationalController.setInverted(getDescriptor().RotationalInverted);

        getDescriptor().TranslationalController.setSmartCurrentLimit((40));
        getDescriptor().RotationalController.setSmartCurrentLimit((30));

        getDescriptor().TranslationalController.enableVoltageCompensation((12D));
        getDescriptor().RotationalController.enableVoltageCompensation((12D));

        TRANSLATIONAL_ENCODER.setPosition((0D));
        TRANSLATIONAL_ENCODER.setMeasurementPeriod((10));
        TRANSLATIONAL_ENCODER.setAverageDepth((2));
        
        getDescriptor().TranslationalController.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1E3D / 1E2D));
        getDescriptor().RotationalController.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1E3D / 1E2D));
      }
      
      getDescriptor().TranslationalController.burnFlash();
      getDescriptor().RotationalController.burnFlash();

      getDescriptor().TranslationalController.setCANTimeout((0));
      getDescriptor().RotationalController.setCANTimeout((0));

      getDescriptor().RotationalEncoder.getConfigurator()
        .apply(new CANcoderConfiguration()
          .withMagnetSensor(new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)));
      getDescriptor().RotationalEncoder.getAbsolutePosition()
        .setUpdateFrequency(StandardRegister.getInstance().getFrequency() / (20));
      getDescriptor().RotationalEncoder.optimizeBusUtilization();

      getDescriptor().RotationalFeedback.continuous(
        VecBuilder.fill(
          -Figures.PI, 
          +Figures.PI
      ));
    }
  }

  @Override
  public synchronized void cease() {
    getDescriptor().TranslationalController.stopMotor();
    getDescriptor().RotationalController.stopMotor();
  }

  @Override
  public synchronized void close() {
    getDescriptor().TranslationalController.close();
    getDescriptor().RotationalController.close();

    getDescriptor().RotationalEncoder.close();   

    UPDATE_TIMESTAMPS.clear();
    TRANSLATIONAL_POSITIONS.clear();
    ROTATIONAL_POSITIONS.clear();
  }

  @Override
  public synchronized void update(final org.frc5411.lib.pattern.Report<@NonNull SwerveModulePosition> Record) {
    final var Article = (Report) Record;

    synchronized(Article) {
      Article.setTranslationalVoltage(getDescriptor().TranslationalController.getBusVoltage() * getDescriptor().TranslationalController.getAppliedOutput());
      Article.setTranslationalAmperage(getDescriptor().TranslationalController.getOutputCurrent());
      Article.setTranslationalVelocity(TRANSLATIONAL_ENCODER.getVelocity() / getDescriptor().TranslationalReduction * getDescriptor().Radius);
      Article.setTranslationalConnected(getDescriptor().TranslationalController.getLastError().equals(REVLibError.kOk));

      Article.setRotationalVoltage(getDescriptor().RotationalController.getBusVoltage() * getDescriptor().RotationalController.getAppliedOutput());
      Article.setRotationalAmperage(getDescriptor().RotationalController.getOutputCurrent());
      Article.setRotationalVelocity(getDescriptor().RotationalEncoder.getVelocity().refresh().getValue());
      Article.setRotationalConnected(getDescriptor().RotationalController.getLastError().equals(REVLibError.kOk)); 

      Article.setConnected(Article.isTranslationalConnected() && Article.isRotationalConnected());
      Article.setOutput(new SwerveModuleState(
        Article
          .getTranslationalVelocity(),
        Rotation2d
          .fromRotations(getDescriptor().RotationalEncoder
            .getAbsolutePosition()
            .refresh()
            .getValue())
          .minus(getDescriptor().RotationalOffset)
      ));

      final double[] Translations, Rotations;
      synchronized(TRANSLATIONAL_POSITIONS) {
        Translations = TRANSLATIONAL_POSITIONS
          .stream()
          .mapToDouble((Position) -> 
            Position.orElse(Double.NaN).doubleValue())
          .toArray();
        TRANSLATIONAL_POSITIONS.clear();
      }
      synchronized(ROTATIONAL_POSITIONS) {
        Rotations = ROTATIONAL_POSITIONS
          .stream()
          .mapToDouble((Position) -> 
            Position.orElse(Double.NaN).doubleValue())
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
          (Translations[Index] - getDescriptor().TranslationalOffset) / getDescriptor().TranslationalReduction * getDescriptor().Radius, 
          Rotation2d
            .fromRotations(Rotations[Index])
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
    getDescriptor().TranslationalController.set(MathUtil.clamp(Demand, (-12D), (12D)));
  }

  /**
   * Mutates the current voltage applied to the module's rotational motor controller
   * @param Demand Voltage sent to the controller object 
   */
  protected void setRotationalVoltage(final double Demand) {
    getDescriptor().RotationalController.set(MathUtil.clamp(Demand, (-12D), (12D)));
  }
}
