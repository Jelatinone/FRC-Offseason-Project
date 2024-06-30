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
package org.frc5411.lib.pattern.actuator;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.pattern.Component;

import edu.wpi.first.util.struct.StructSerializable;

import java.util.Optional;

import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Actuator</h1>
 * 
 * <p>Describes any {@link Component} that can also be actuated, like a motor or pneumatic device. This is essentially any device that can be
 * actuated, i.e. controlling the Effort of the controller. This provides base implementation for essentially any system that can be moved 
 * (but not limited to) to a given Demand, such as a Drivebase, Piston, or Module.
 * 
 * @author Cody Washington
 */
@SuppressWarnings("unchecked")
public interface Actuator<@NonNull Reference extends StructSerializable, @NonNull Measurement extends StructSerializable> extends Component<Measurement> {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Mutates the current demand state of the actuator to a different state, but does not immediately process the correct actuator effort required
   * to reach this demand state, instead this is done during {@link #periodic()}. Returns, if needed an 'optimized' instance of the Demand
   * @param Demand Reference state to reach
   * @return Optimized state, used by the internal controller and actuator, default behavior returns the same instance as provided
   */
  default Reference set(final @NonNull Reference Demand) {
    ((Report<Reference,Measurement>) getReport()).setState(Demand);
    return Demand;
  }

  /**
   * Immediately stops the actuation of hardware; such that calls to {@link #set(StructSerializable)} can still be made. Used as a 
   * safety feature to prevent unsafe actuator movements.
   */
  void cease();

  /**
   * Provides the current reference state, in other words the Demand set by calling {@link #set(StructSerializable)}.
   * @return Struct of controller state (reference)
   */
  default Optional<Reference> getState() {
    return Optional.ofNullable(((Report<Reference,Measurement>) getReport()).getState());
  }

  /**
   * Provides the current controller input to the actuator, in other words the controller effort updated internally each {@link #periodic()} call.
   * @return Struct of controller input 
   */
  default Optional<Reference> getInput() {
    return Optional.ofNullable(((Report<Reference,Measurement>) getReport()).getInput());
  }


  /**
   * Provides the current controller output of the actuator, in other words the controller's feedback updated internally each {@link #periodic()} call.
   * @return Struct of controller output 
   */
  default Optional<Reference> getOutput() {
    return Optional.ofNullable(((Report<Reference,Measurement>) getReport()).getOutput());
  }
}
