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
package org.frc5411.robot2024.subsystems.drivebase;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//

import org.frc5411.lib.annotation.Unit;
import org.frc5411.lib.annotation.Unit.Measured;
import org.frc5411.lib.control.archetype.PIDConstants;
import org.frc5411.lib.instrument.module.Limit;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Optional;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>
 * 
 * @see DrivebaseSubsystem
 * @author Cody Washington
 * 
 */
@FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
public class Constants {
  //----------------------------------------------------------------------[Internal]---------------------------------------------------------------------------//
  /**
   * <h1>Module</h1>
   * 
   */
  @FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
  public enum Module {
    FRONT_LEFT(
      Descriptions.REAL_MODULE_DESCRIPTOR
        .TranslationalController(new CANSparkMax((11), MotorType.kBrushless))
        .RotationalOffset(Rotation2d.fromRotations((0.724121D)))
        .RotationalEncoder(new CANcoder((31),("CTREBUS")))
        .RotationalController(new CANSparkMax((21), MotorType.kBrushless))
        .Position(new Translation2d((Identity.ROBOT_WIDTH)  / (2), (Identity.ROBOT_LENGTH) / (2))),
      Descriptions.MOCK_MODULE_DESCRIPTOR
        .TranslationalController(new DCMotorSim(DCMotor.getNEO((1)), (6.75D), (0.025D)))
        .RotationalOffset(Rotation2d.fromRotations(Math.random()))
        .RotationalEncoder(Optional.empty())
        .RotationalController((new DCMotorSim(DCMotor.getNEO((1)), ((150D) / (7D)), (0.004D))))
        .Position(new Translation2d((Identity.ROBOT_WIDTH)  / (2), (Identity.ROBOT_LENGTH) / (2)))),
    FRONT_RIGHT(
      Descriptions.REAL_MODULE_DESCRIPTOR
        .TranslationalController(new CANSparkMax((12), MotorType.kBrushless))
        .RotationalOffset(Rotation2d.fromRotations((0.726074D)))
        .RotationalEncoder(new CANcoder((32)))
        .RotationalController(new CANSparkMax((22), MotorType.kBrushless))
        .Position(new Translation2d((Identity.ROBOT_WIDTH)  / (2), (Identity.ROBOT_LENGTH) / (2))),
      Descriptions.MOCK_MODULE_DESCRIPTOR
        .TranslationalController(new DCMotorSim(DCMotor.getNEO((1)), (6.75D), (0.025D)))
        .RotationalOffset(Rotation2d.fromRotations(Math.random()))
        .RotationalEncoder(Optional.empty())
        .RotationalController((new DCMotorSim(DCMotor.getNEO((1)), ((150D) / (7D)), (0.004D))))
        .Position(new Translation2d((Identity.ROBOT_WIDTH)  / (2), (Identity.ROBOT_LENGTH) / (2)))),
    REAR_LEFT(
      Descriptions.REAL_MODULE_DESCRIPTOR
        .TranslationalController(new CANSparkMax((13), MotorType.kBrushless))
        .RotationalOffset(Rotation2d.fromRotations((0.609863D)))
        .RotationalEncoder(new CANcoder((33)))
        .RotationalController(new CANSparkMax((23), MotorType.kBrushless))
        .Position(new Translation2d( (Identity.ROBOT_WIDTH)  / (2), (Identity.ROBOT_LENGTH) / (2))),
      Descriptions.MOCK_MODULE_DESCRIPTOR
        .TranslationalController(new DCMotorSim(DCMotor.getNEO((1)), (6.75D), (0.025D)))
        .RotationalOffset(Rotation2d.fromRotations(Math.random()))
        .RotationalEncoder(Optional.empty())
        .RotationalController((new DCMotorSim(DCMotor.getNEO((1)), ((150D) / (7D)), (0.004D))))
        .Position(new Translation2d((Identity.ROBOT_WIDTH)  / (2), (Identity.ROBOT_LENGTH) / (2)))),
    REAR_RIGHT(
      Descriptions.REAL_MODULE_DESCRIPTOR
        .TranslationalController(new CANSparkMax((14), MotorType.kBrushless))
        .RotationalOffset(Rotation2d.fromRotations((0.382568D)))
        .RotationalEncoder(new CANcoder((34)))
        .RotationalController(new CANSparkMax((24), MotorType.kBrushless))
        .Position(new Translation2d((Identity.ROBOT_WIDTH)  / (2), (Identity.ROBOT_LENGTH) / (2))),
      Descriptions.MOCK_MODULE_DESCRIPTOR
        .TranslationalController(new DCMotorSim(DCMotor.getNEO((1)), (6.75D), (0.025D)))
        .RotationalOffset(Rotation2d.fromRotations(Math.random()))
        .RotationalEncoder(Optional.empty())
        .RotationalController((new DCMotorSim(DCMotor.getNEO((1)), ((150D) / (7D)), (0.004D))))
        .Position(new Translation2d((Identity.ROBOT_WIDTH)  / (2), (Identity.ROBOT_LENGTH) / (2))));

    org.frc5411.lib.instrument.module.Descriptor<CANSparkBase,CANcoder> REAL_DESCRIPTOR;
    org.frc5411.lib.instrument.module.Descriptor<DCMotorSim,Optional<Object>> MOCK_DESCRIPTOR;

    /**
     * Module Constructor.
     * @param Real {@link org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder descriptor builder} which contains the relevant module constants
     *              for a real module to be constructed
     * @param Mock {@link org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder descriptor builder} which contains the relevant module constants
     *              for a mock module to be constructed
     * @implSpec Each module enum constant, {@code FRONT_LEFT}; {@code FRONT_RIGHT}; etc, should use the provided base 
     * {@link org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder descriptor builders} from {@link descriptions} and use 
     * {@link org.frc5411.lib.instrument.module.Descriptor#clone() Descriptor.clone()} to specify it's own descriptor specific to it's emplacement on the chassis
     */
    Module(
      final org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<CANSparkBase,CANcoder> Real, 
      final org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<DCMotorSim,Optional<Object>> Mock) {
      REAL_DESCRIPTOR = Real.Identity(this).build();
      MOCK_DESCRIPTOR = Mock.Identity(this).build();
    }

    /**
     * Provides the related descriptor for this enum constant's real-world counterpart module to be constructed
     * @return Real-world descriptor of this module
     */
    public org.frc5411.lib.instrument.module.Descriptor<CANSparkBase,CANcoder> getRealDescriptor() {
      return REAL_DESCRIPTOR;
    }

    /**
     * Provides the related descriptor for this enum constant's Mock counterpart module to be constructed
     * @return Mock descriptor of this module
     */
    public org.frc5411.lib.instrument.module.Descriptor<DCMotorSim,Optional<Object>> getMockDescriptor() {
      return MOCK_DESCRIPTOR;
    }
  }

  /**
   * <h1>Identity<h1>
   * 
   */
  @FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
  public class Identity {
    @Unit(measures = Measured.DISTANCE, symbol = ("meters"))
    static Double ROBOT_WIDTH = Units.inchesToMeters((24.6D));
    @Unit(measures = Measured.DISTANCE, symbol = ("meters"))
    static Double ROBOT_LENGTH = Units.inchesToMeters((24.6D));
    @Unit(measures = Measured.DISTANCE, symbol = ("meters"))
    static Double ROBOT_RADIUS_METERS = Math.hypot(ROBOT_LENGTH / (2d), ROBOT_WIDTH / (2d));

    @Unit(measures = Measured.DISTANCE, symbol = ("meters/second"))
    static Double ROBOT_MAXIMUM_LINEAR_VELOCITY = Units.feetToMeters((19.1D));
    @Unit(measures = Measured.DISTANCE, symbol = ("radians/second"))
    static Double ROBOT_MAXIMUM_ANGULAR_VELOCITY = ROBOT_MAXIMUM_LINEAR_VELOCITY / ROBOT_RADIUS_METERS;
  }
}
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
/**
 * <h1>Descriptions</h1>
 * 
 * <p>Contains base-level {@link org.frc5411.lib.pattern.Descriptor descriptors} for the related devices for the {@link DrivebaseSubsystem}, namely
 * it's {@link Module modules} and {@link org.frc5411.lib.instrument.gyroscope.Gyroscope gyroscopic} hardware.
 */
@FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
class Descriptions {

  static final org.frc5411.lib.instrument.gyroscope.Descriptor.DescriptorBuilder GYROSCOPE_DESCRIPTOR_BUILDER = 
    org.frc5411.lib.instrument.gyroscope.Descriptor.builder()
      .Identity((0))
      .Offset(VecBuilder.fill(
        (0D), 
        (0D), 
        (0D)));

  static final org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<CANSparkBase,CANcoder> REAL_MODULE_DESCRIPTOR = 
    org.frc5411.lib.instrument.module.Descriptor.<CANSparkBase,CANcoder>builder()
      .TranslationalReduction((6.75D))
      .TranslationalOffset((0D))
      .TranslationalInverted((false))
      .TranslationalFeedback(
        PIDConstants.builder()
          .Proportional((0D))
          .Integral((0D))
          .Derivative((0D))
          .build().toController())
      .RotationalReduction((150D) / (7D))
      .RotationalInverted((false))
      .RotationalFeedback(
        PIDConstants.builder()
          .Proportional((0D)) 
          .Integral((0D))
          .Derivative((0D))
          .build().toController())
      .Radius(Units.inchesToMeters((4D)))
      .Limits(new Limit(
        (0D), 
        (0D), 
        (0D)));

  static final org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<DCMotorSim,Optional<Object>> MOCK_MODULE_DESCRIPTOR =
    org.frc5411.lib.instrument.module.Descriptor.<DCMotorSim,Optional<Object>>builder()
      .TranslationalReduction((6.75D))
      .TranslationalOffset((0D))
      .TranslationalInverted((false))
      .TranslationalFeedback(
        PIDConstants.builder()
          .Proportional((0D))
          .Integral((0D))
          .Derivative((0D))
          .build().toController())
      .RotationalReduction((150D) / (7D))
      .RotationalInverted((false))
      .RotationalFeedback(
        PIDConstants.builder()
          .Proportional((0D))
          .Integral((0D))
          .Derivative((0D))
          .build().toController())
      .Radius(Units.inchesToMeters((4D)))
      .Limits(new Limit(
        (0D), 
        (0D), 
        (0D)));
}