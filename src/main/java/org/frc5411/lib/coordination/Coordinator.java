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
package org.frc5411.lib.coordination;
//-------------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//

//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 * <h1>Coordinator</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FunctionalInterface
public interface Coordinator<Coordinated> {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Updates the underlying controller behavior, and in turn produces a resulting {@code Coordinated} value for upon which actuators should apply to reach
   * a desired end-point.
   * @return Coordinated value, to be applied to actuators
   */
  Coordinated update();

  /**
   * Resets this controller to a given initial fields and all other relevant data to their default states; such that calls must be made to the relevant
   * mutators.
   * @implSpec By default, the behavior of this method performs no actions.
   */
  default void reset() {}
}
