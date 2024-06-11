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
@Builder(toBuilder = true, setterPrefix = ("with"))
@FieldDefaults(level = AccessLevel.PROTECTED, makeFinal = (true))
public class Descriptor<@NonNull Actuator, @NonNull Encoder> extends org.frc5411.lib.pattern.Descriptor<Component<SwerveModulePosition>> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Double TranslationalReduction;
  Double TranslationalOffset;
  Double TranslationalVelocity;
  Double TranslationalAcceleration;
  Boolean TranslationalInverted;
  Actuator TranslationalController;

  Controller<N2,N1,N1> TranslationalFeedback;
  
  Double RotationalReduction;
  Double RotationalOffset;
  Double RotationalVelocity;
  Boolean RotationalInverted;
  Encoder RotationalEncoder;
  Actuator RotationalController;
  Controller<N2,N1,N1> RotationalFeedback;

  Double Radius;
  Enum<?> Placement;

  @Override
  public Descriptor<Actuator,Encoder> clone() {
    return new Descriptor<>(
      TranslationalReduction,
      TranslationalOffset,
      TranslationalVelocity,
      TranslationalAcceleration,
      TranslationalInverted,
      TranslationalController,
      TranslationalFeedback,
      RotationalReduction,
      RotationalOffset,
      RotationalVelocity,
      RotationalInverted,
      RotationalEncoder,
      RotationalController,
      RotationalFeedback,
      Radius,
      Placement
    );
  }
}
