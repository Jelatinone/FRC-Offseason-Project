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
package org.frc5411.robot2024.subsystems.vision;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.Supplier;

import org.frc5411.robot2024.Manager;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Constants</h1>
 * 
 * @see VisionSubsystem
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
public class Constants {  
  //-----------------------------------------------------------------------[Enums]-----------------------------------------------------------------------------//
  /**
   * <h1>Camera</h1>
   * 
   * @implNote Enum Constants are named {LOCATION}${SIDE} to prevent AdvantageScope from folding the tabs...
   */
  @FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
  public enum Camera implements Supplier<org.frc5411.lib.instrument.camera.Descriptor<?>> {
    //---------------------------------------------------------------------[Values]----------------------------------------------------------------------------//
    FRONT$LEFT(
      RobotBase.isReal()?
      Descriptions.REAL_CAMERA_DESCRIPTOR
        .Hardware(NetworkTableInstance.getDefault().getTable(("LLLeft")))
        .Position(new Transform3d(new Translation3d((3.5e-1D), (3.2e-1D), (3.3e-1D)), new Rotation3d((0D), (-4.45059e-1D), (-3.351032e-1D)))):
      Descriptions.MOCK_CAMERA_DESCRIPTOR
        .Position(new Transform3d())),
    FRONT$RIGHT(
      RobotBase.isReal()?
      Descriptions.REAL_CAMERA_DESCRIPTOR
        .Hardware(NetworkTableInstance.getDefault().getTable(("LLRight")))
        .Position(new Transform3d(new Translation3d((3.5e-1D), -(3.2e-1D), (3.3e-1D)), new Rotation3d((0D), (-4.45059e-1D), (2.565634e-1D)))):
      Descriptions.MOCK_CAMERA_DESCRIPTOR
        .Position(new Transform3d()));
    //-----------------------------------------------------------------------[Constants]-----------------------------------------------------------------------//
    org.frc5411.lib.instrument.camera.Descriptor<?> DESCRIPTOR;
    //---------------------------------------------------------------------[Constructor(s)]--------------------------------------------------------------------//
    /**
     * Camera Constructor.
     * @param Descriptor {@link org.frc5411.lib.instrument.camera.Descriptor.DescriptorBuilder descriptor builder} which contains the relevant camera constants
     *              for a real or mock camera to be constructed
     * @implSpec Each camera enum constant, {@code FRONT_LEFT}; {@code FRONT_RIGHT}; etc., should use the provided base
     * {@link org.frc5411.lib.instrument.camera.Descriptor.DescriptorBuilder descriptor builders} from {@link Descriptions} and use
     * {@link org.frc5411.lib.instrument.camera.Descriptor#clone() Descriptor.clone()} to specify its own descriptor specific to its emplacement on the chassis
     */
    Camera(final org.frc5411.lib.instrument.camera.Descriptor.DescriptorBuilder<?> Descriptor) {
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
    public org.frc5411.lib.instrument.camera.Descriptor<?> get() {
      return DESCRIPTOR;
    }
  }
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//

  //-----------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   * <h1>Identity<h1>
   */
  @FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
  public static class Identity {
    
  }
}
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
/**
 * <h1>Descriptions</h1>
 * 
 * <p>Contains base-level {@link org.frc5411.lib.pattern.Descriptor descriptors} for the related devices for the {@link VisionSubsystem}, namely
 * it's Cameras.
 */
@FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
class Descriptions {

  static org.frc5411.lib.instrument.camera.Descriptor.DescriptorBuilder<NetworkTable> REAL_CAMERA_DESCRIPTOR = 
    org.frc5411.lib.instrument.camera.Descriptor.<NetworkTable>builder();

  static org.frc5411.lib.instrument.camera.Descriptor.DescriptorBuilder<Supplier<Pose2d>> MOCK_CAMERA_DESCRIPTOR =
    org.frc5411.lib.instrument.camera.Descriptor.<Supplier<Pose2d>>builder()
      .Hardware(() -> Manager.getInstance().getVehicleOdometry());
}