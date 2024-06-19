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
import org.frc5411.lib.control.archetype.PIDConstants;
import org.frc5411.lib.control.archetype.PIDController;
import org.frc5411.lib.instrument.module.Limit;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

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
@FieldDefaults(level = AccessLevel.PUBLIC, makeFinal = (true))
public class Constants {
  //----------------------------------------------------------------------[Internal]---------------------------------------------------------------------------//
  @FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
  public enum Module {

    FRONT_LEFT(
      (null),
      (null)),

    FRONT_RIGHT(
      (null),
      (null)),

    REAR_LEFT(
      (null),
      (null)),

    REAR_RIGHT(
      (null),
      (null));

    org.frc5411.lib.instrument.module.Descriptor<CANSparkBase,CANcoder> REAL_DESCRIPTOR;
    org.frc5411.lib.instrument.module.Descriptor<DCMotorSim,Supplier<Number>> MOCK_DESCRIPTOR;

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
      final org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<DCMotorSim,Supplier<Number>> Mock) {
      REAL_DESCRIPTOR = Real.withPlacement(this).build();
      MOCK_DESCRIPTOR = Mock.withPlacement(this).build();
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
    public org.frc5411.lib.instrument.module.Descriptor<DCMotorSim,Supplier<Number>> getMockDescriptor() {
      return MOCK_DESCRIPTOR;
    }
  }
}
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
/**
 * <h1>Descriptions</h1>
 * 
 * <p>Contains base-level {@link org.frc5411.lib.pattern.Descriptor descriptors} for the related devices for the {@link DrivebaseSubsystem}, namely
 * it's {@link Module modules} and gyroscopic hardware.
 */
@FieldDefaults(level = AccessLevel.PUBLIC, makeFinal = (true))
class Descriptions {

  static final org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<CANSparkBase,CANcoder> REAL_MODULE_DESCRIPTOR = 
  org.frc5411.lib.instrument.module.Descriptor.<CANSparkBase,CANcoder>builder()
    .withTranslationalReduction((6.12D))
    .withTranslationalOffset(Units.inchesToMeters((0D)))
    .withTranslationalInverted((false))
    .withTranslationalFeedback(new PIDController(
      PIDConstants.builder()
        .setProportional((0D))
        .setIntegral((0D))
        .setDerivative((0D))
        .build()))
    .withRotationalReduction((150D) / (7D))
    .withRotationalInverted((false))
    .withRotationalFeedback(
      PIDConstants.builder()
        .setProportional((0D))
        .setIntegral((0D))
        .setDerivative((0D))
        .build().toController())
    .withRadius(Units.inchesToMeters((4D)))
    .withLimits(new Limit(
      (0D), 
      (0D), 
      (0D)));

  static final org.frc5411.lib.instrument.module.Descriptor.DescriptorBuilder<DCMotorSim,Supplier<Number>> MOCK_MODULE_DESCRIPTOR =
    org.frc5411.lib.instrument.module.Descriptor.<DCMotorSim,Supplier<Number>>builder()
      .withTranslationalReduction((6.12D))
      .withTranslationalOffset(Units.inchesToMeters((0D)))    
      .withTranslationalInverted((false))
      .withTranslationalFeedback(new PIDController(
        PIDConstants.builder()
          .setProportional((0D))
          .setIntegral((0D))
          .setDerivative((0D))
          .build()))
      .withRotationalReduction((150D) / (7D))
      .withRotationalInverted((false))
      .withRotationalFeedback(
        PIDConstants.builder()
          .setProportional((0D))
          .setIntegral((0D))
          .setDerivative((0D))
          .build().toController())
      .withRadius(Units.inchesToMeters((4D)))
      .withLimits(new Limit(
        (0D), 
        (0D), 
        (0D)));
}