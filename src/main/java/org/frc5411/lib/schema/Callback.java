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
package org.frc5411.lib.schema;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.utility.Aggregator;

import edu.wpi.first.hal.HALUtil;

import java.util.Objects;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Callback</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class Callback {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Runnable PROCEDURE;
  Aggregator<Double> DISCRETE_AGGREGATOR;
  Double PERIOD;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Callback Constructor.
   * @param Procedure Runnable operation that requires periodic calls to itself
   * @param Period    Period on which to perform the procedure
   */
  public Callback(final Runnable Procedure, final Double Period) {
    PROCEDURE = Objects.requireNonNull(Procedure);
    PERIOD = Period;
    DISCRETE_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1E6D, 
      (Previous, Current) -> Current - Previous);
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Tries (attempts) to perform the underlying operation if the difference in time since last operation it is greater than, or equal to the period on which
   * this operation should occur. 
   */
  public synchronized void attempt() {
    synchronized(this) {
      if(DISCRETE_AGGREGATOR.acquire() >= PERIOD) {
        DISCRETE_AGGREGATOR.retain(
          DISCRETE_AGGREGATOR.getRetained() + DISCRETE_AGGREGATOR.getAggregated());
        PROCEDURE.run();
      }
    }
  }
}