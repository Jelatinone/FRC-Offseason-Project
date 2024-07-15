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
import org.frc5411.lib.coordination.archetype.HeadingCoordinator;
import org.frc5411.lib.coordination.archetype.TeleoperatedCoordinator;
import org.frc5411.lib.instrument.module.Limit;
import org.frc5411.lib.nascent.archetype.PIDController;
import org.frc5411.lib.nascent.archetype.ProfiledPIDController;
import org.frc5411.lib.utility.Figures;

import org.frc5411.robot2024.Manager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Constants</h1>
 * 
 * @see DrivebaseSubsystem
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
public class Constants {
  //-----------------------------------------------------------------------[Enums]-----------------------------------------------------------------------------//
  /**
   * <h1>Module</h1>
   * 
   * @implNote Enum Constants are named {LOCATION}${SIDE} to prevent AdvantageScope from folding the tabs...
   */
  @FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
  public enum Modules implements Supplier<org.frc5411.lib.instrument.module.Descriptor<?,?>> {
    //---------------------------------------------------------------------[Values]----------------------------------------------------------------------------//
    FRONT$LEFT(
      RobotBase.isReal()?
      Descriptions.REAL_MODULE_DESCRIPTOR
        .TranslationalController(new CANSparkMax((11), MotorType.kBrushless))
        .RotationalOffset(Rotation2d.fromRotations((0.724121D)))
        .RotationalEncoder(new CANcoder((31), ("CTREBUS")))
        .RotationalController(new CANSparkMax((21), MotorType.kBrushless))
        .Position(new Translation2d((Identity.WIDTH)  / (2), (Identity.LENGTH) / (2))):
      Descriptions.MOCK_MODULE_DESCRIPTOR
        .TranslationalController(new DCMotorSim(DCMotor.getNEO((1)), (6.75D), (0.025D)))
        .RotationalOffset(Rotation2d.fromRotations(Math.random()))
        .RotationalController((new DCMotorSim(DCMotor.getNEO((1)), ((150D) / (7D)), (0.004D))))
        .Position(new Translation2d((Identity.WIDTH)  / (2), (Identity.LENGTH) / (2)))),
    FRONT$RIGHT(
      RobotBase.isReal()?
      Descriptions.REAL_MODULE_DESCRIPTOR
        .TranslationalController(new CANSparkMax((12), MotorType.kBrushless))
        .RotationalOffset(Rotation2d.fromRotations((0.726074D)))
        .RotationalEncoder(new CANcoder((32), ("CTREBUS")))
        .RotationalController(new CANSparkMax((22), MotorType.kBrushless))
        .Position(new Translation2d((Identity.WIDTH)  / (2), -(Identity.LENGTH) / (2))):
      Descriptions.MOCK_MODULE_DESCRIPTOR
        .TranslationalController(new DCMotorSim(DCMotor.getNEO((1)), (6.75D), (0.025D)))
        .RotationalOffset(Rotation2d.fromRotations(Math.random()))
        .RotationalController((new DCMotorSim(DCMotor.getNEO((1)), ((150D) / (7D)), (0.004D))))
        .Position(new Translation2d((Identity.WIDTH)  / (2), -(Identity.LENGTH) / (2)))),
    REAR$LEFT(
      RobotBase.isReal()?
      Descriptions.REAL_MODULE_DESCRIPTOR
        .TranslationalController(new CANSparkMax((13), MotorType.kBrushless))
        .RotationalOffset(Rotation2d.fromRotations((0.609863D)))
        .RotationalEncoder(new CANcoder((33), ("CTREBUS")))
        .RotationalController(new CANSparkMax((23), MotorType.kBrushless))
        .Position(new Translation2d(-(Identity.WIDTH)  / (2), (Identity.LENGTH) / (2))):
      Descriptions.MOCK_MODULE_DESCRIPTOR
        .TranslationalController(new DCMotorSim(DCMotor.getNEO((1)), (6.75D), (0.025D)))
        .RotationalOffset(Rotation2d.fromRotations(Math.random()))
        .RotationalController((new DCMotorSim(DCMotor.getNEO((1)), ((150D) / (7D)), (0.004D))))
        .Position(new Translation2d(-(Identity.WIDTH)  / (2), (Identity.LENGTH) / (2)))),
    REAR$RIGHT(
      RobotBase.isReal()?
      Descriptions.REAL_MODULE_DESCRIPTOR
        .TranslationalController(new CANSparkMax((14), MotorType.kBrushless))
        .RotationalOffset(Rotation2d.fromRotations((0.382568D)))
        .RotationalEncoder(new CANcoder((34), ("CTREBUS")))
        .RotationalController(new CANSparkMax((24), MotorType.kBrushless))
        .Position(new Translation2d(-(Identity.WIDTH)  / (2), -(Identity.LENGTH) / (2))):
      Descriptions.MOCK_MODULE_DESCRIPTOR
        .TranslationalController(new DCMotorSim(DCMotor.getNEO((1)), (6.75D), (0.025D)))
        .RotationalOffset(Rotation2d.fromRotations(Math.random()))
        .RotationalController((new DCMotorSim(DCMotor.getNEO((1)), ((150D) / (7D)), (0.004D))))
        .Position(new Translation2d(-(Identity.WIDTH)  / (2), -(Identity.LENGTH) / (2))));
    //-----------------------------------------------------------------------[Constants]-----------------------------------------------------------------------//
    org.frc5411.lib.instrument.module.Descriptor<?,?> DESCRIPTOR;
    //---------------------------------------------------------------------[Constructor(s)]--------------------------------------------------------------------//
    /**
     * Module Constructor.
     * @param Descriptor {@link org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder descriptor builder} which contains the relevant module constants
     *              for a real or mock module to be constructed
     * @implSpec Each module enum constant, {@code FRONT_LEFT}; {@code FRONT_RIGHT}; etc., should use the provided base
     * {@link org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder descriptor builders} from {@link Descriptions} and use
     * {@link org.frc5411.lib.instrument.module.Descriptor#clone() Descriptor.clone()} to specify its own descriptor specific to its emplacement on the chassis
     */
    Modules(final org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<?,?> Descriptor) {
      DESCRIPTOR = Descriptor
        .Identity(this)
        .build();
    }
    //-----------------------------------------------------------------------[Accessors]-----------------------------------------------------------------------//
    /**
     * Provides the descriptor of this enum constant's stored value, which at runtime via {@link RobotBase#isReal()} determines the correct (real or mock) descriptor
     * to use.
     * @return Descriptor based on if the robot is real or simulated
     */
    @Override
    public org.frc5411.lib.instrument.module.Descriptor<?,?> get() {
      return DESCRIPTOR;
    }
  }
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  static org.frc5411.lib.instrument.gyroscope.Descriptor<?> GYROSCOPE_DESCRIPTOR = Descriptions.REAL_GYROSCOPE_DESCRIPTOR.build();  
  static org.frc5411.lib.nascent.archetype.ProfiledPIDController.Descriptor HEADING_COORDINATOR_DESCRIPTOR = Descriptions.HEADING_COORDINATOR_DESCRIPTION.build();
  //-----------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   * <h1>Identity<h1>
   */
  @FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
  public static class Identity {
    static Double WIDTH = Units.inchesToMeters((24.25D));
    static Double LENGTH = Units.inchesToMeters((24.25D));
    static Double RADIUS = Math.hypot(LENGTH / (2d), WIDTH / (2d));

    static Double LINEAR_VELOCITY = (4.8D);
    static Double LINEAR_ACCELERATION = LINEAR_VELOCITY * (5D);
    static Double ANGULAR_VELOCITY = LINEAR_VELOCITY / RADIUS;

    static Pose2d PRESET = new Pose2d(); 
  }

  /**
   * <h1>Regulation<h1>
   */
  @FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
  public static class Regulation {
    static Translation2d[] LOCATIONS = Stream.of(Modules.values()).map((Module) -> Module.get().Position).toArray(Translation2d[]::new);
    static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);    

    static Limit LIMITS = new Limit(Identity.LINEAR_VELOCITY, Identity.LINEAR_ACCELERATION, Identity.ANGULAR_VELOCITY);

    static HeadingCoordinator HEADING_COORDINATOR = new HeadingCoordinator(new ProfiledPIDController(HEADING_COORDINATOR_DESCRIPTOR), () -> Manager.getInstance().getVehicleOdometry().getRotation());
    static TeleoperatedCoordinator TELEOPERATED_COORDINATOR = new TeleoperatedCoordinator((0D), LIMITS);    
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

  static org.frc5411.lib.nascent.archetype.ProfiledPIDController.Descriptor.DescriptorBuilder HEADING_COORDINATOR_DESCRIPTION = 
    org.frc5411.lib.nascent.archetype.ProfiledPIDController.Descriptor.builder()
      .Proportional((9D))
      .Integral((0D))
      .Derivative((5E-1D))
      .Velocity((8D))
      .Acceleration((20D));

  static org.frc5411.lib.instrument.gyroscope.Descriptor.DescriptorBuilder<?> REAL_GYROSCOPE_DESCRIPTOR = 
    org.frc5411.lib.instrument.gyroscope.Descriptor.<Pigeon2>builder()
      .Identity((0))
      .Hardware(new Pigeon2((0)))
      .Offset(new Rotation3d());

  static org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<CANSparkBase,CANcoder> REAL_MODULE_DESCRIPTOR = 
    org.frc5411.lib.instrument.module.Descriptor.<CANSparkBase,CANcoder>builder()
      .TranslationalReduction((6.75D))
      .TranslationalOffset((0D))
      .TranslationalInverted((false))
      .TranslationalFeedback(PIDController.Descriptor.builder().Proportional((0.2D)).Integral((0D)).Derivative((5E-1D)).build().<PIDController.Descriptor,PIDController>complete(PIDController::new))
      .RotationalReduction((150D) / (7D))
      .RotationalInverted((false))
      .RotationalFeedback(PIDController.Descriptor.builder().Proportional((2.81D)).Integral((0D)).Derivative((5E-1D)).build().<PIDController.Descriptor,PIDController>complete(PIDController::new))
      .Radius(Units.inchesToMeters((4D)))
      .Limits(Limit.builder().TranslationalVelocity((4.8D)).TranslationalAcceleration((4.8D) * (5D)).RotationalVelocity((24) * Figures.PI).build());

  static org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<DCMotorSim,Optional<Object>> MOCK_MODULE_DESCRIPTOR =
    org.frc5411.lib.instrument.module.Descriptor.<DCMotorSim,Optional<Object>>builder()
      .TranslationalReduction((6.75D))
      .TranslationalOffset((0D))
      .TranslationalInverted((false))
      .TranslationalFeedback(PIDController.Descriptor.builder().Proportional((0.35D)).Integral((0D)).Derivative((0D)).build().<PIDController.Descriptor,PIDController>complete(PIDController::new))
      .RotationalReduction((150D) / (7D))
      .RotationalInverted((false))
      .RotationalEncoder(Optional.empty())
      .RotationalFeedback(PIDController.Descriptor.builder().Proportional((4.05e1D)).Integral((0D)).Derivative((0D)).build().<PIDController.Descriptor,PIDController>complete(PIDController::new))
      .Radius(Units.inchesToMeters((4D)))
      .Limits(Limit.builder().TranslationalVelocity((4.8D)).TranslationalAcceleration((4.8D) * (5D)).RotationalVelocity((24) * Figures.PI).build());
}