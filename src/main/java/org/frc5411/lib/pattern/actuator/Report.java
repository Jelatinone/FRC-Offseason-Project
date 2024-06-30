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
import edu.wpi.first.util.struct.StructSerializable;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.NonNull;
import lombok.Setter;
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
@NonNull
@Getter
@Setter
public abstract class Report<@NonNull Reference extends StructSerializable, @NonNull Measurement extends StructSerializable> extends org.frc5411.lib.pattern.Report<Measurement> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  /**
   * -- GETTER --
   * Value used by the {@link Component component's} internal hardware-level controller as a reference or 'setpoint' value
   * @return Controller's state (reference, 'setpoint')
   */
  volatile Reference State; 
  /**
   * -- GETTER --
   * Value produced by the {@link Component component's} internal hardware-level controller and applied to the hardware
   * @return Controller's output (hardware's input)
   */
  volatile Reference Input;
  /**
   * -- GETTER --
   * Value produced by the {@link Component component's} hardware, and used as feedback for it's internal hardware-level controller
   * @return Controller's input (hardware's output)
   */
  volatile Reference Output;
}
