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
package org.frc5411.lib.pattern.actuator.module;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.annotation.Unit;
import org.frc5411.lib.annotation.Unit.Measured;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.AutoLog;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Report</h1>
 * 
 * @see org.frc5411.lib.pattern.Report Report
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PROTECTED)
@Getter
@AutoLog
public class Report extends org.frc5411.lib.pattern.actuator.Report<SwerveModuleState,SwerveModulePosition> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  @Unit(measures = Measured.VELOCITY)
  volatile double TranslationalVelocity;

  @Unit(measures = Measured.VOLTAGE)
  volatile double TranslationalVoltage;

  @Unit(measures = Measured.AMPERAGE)
  volatile double TranslationalAmperage;
  volatile boolean TranslationalConnected;

  @Unit(measures = Measured.VELOCITY)
  volatile double RotationalVelocity;

  @Unit(measures = Measured.VOLTAGE)
  volatile double RotationalVoltage;

  @Unit(measures = Measured.AMPERAGE)
  volatile double RotationalAmperage;
  volatile boolean RotationalConnected;  
}