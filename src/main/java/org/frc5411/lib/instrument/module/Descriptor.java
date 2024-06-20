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
import org.frc5411.lib.control.Controller;
import org.frc5411.lib.pattern.Component;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import lombok.AccessLevel;
import lombok.Builder;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Descriptor</h1>
 * 
 * @see org.frc5411.lib.pattern.Descriptor Descriptor
 * 
 * @author Cody Washington
 */
@Builder(toBuilder = (true))
@FieldDefaults(level = AccessLevel.PUBLIC, makeFinal = (true))
public class Descriptor<@NonNull Actuator, @NonNull Encoder> extends org.frc5411.lib.pattern.Descriptor<Component<SwerveModulePosition>> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Double TranslationalReduction;
  Double TranslationalOffset;
  Boolean TranslationalInverted;
  Actuator TranslationalController;
  Controller<N2,N1,N1> TranslationalFeedback;
  
  Double RotationalReduction;
  Rotation2d RotationalOffset;
  Boolean RotationalInverted;
  Encoder RotationalEncoder;
  Actuator RotationalController;
  Controller<N2,N1,N1> RotationalFeedback;

  Translation2d Position;
  Double Radius;
  Enum<?> Identity;
  Limit Limits;
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public Descriptor<Actuator,Encoder> clone() {
    return new Descriptor<>(
      TranslationalReduction,
      TranslationalOffset,
      TranslationalInverted,
      TranslationalController,
      TranslationalFeedback,
      RotationalReduction,
      RotationalOffset,
      RotationalInverted,
      RotationalEncoder,
      RotationalController,
      RotationalFeedback,
      Position,
      Radius,
      Identity,
      Limits
    );
  }
}