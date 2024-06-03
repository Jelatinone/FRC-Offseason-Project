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
package org.frc5411.lib.control.archetype;
//-------------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.FieldDefaults;
//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 * <h1>Component</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */

 @Builder(toBuilder = (true), setterPrefix = ("set"))
 @Getter
 @FieldDefaults(level = AccessLevel.PUBLIC, makeFinal = (true))
public class PIDConstants {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  public final Double ProportionalGain;

  public final Double IntegralGain;

  public final Double DerivativeGain;
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Transforms the relevant PID Constants stored within this object into a 'tuned' controller object
   * @return PID controller object from stored constants
   */
  public PIDController toController() {
    return new PIDController(this);
  }
  
}
