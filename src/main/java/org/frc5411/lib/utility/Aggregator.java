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
package org.frc5411.lib.utility;
//-----------------------------------------------------------------------[Libraries]----------------------------------------------------------------------------//

import lombok.AccessLevel;
import lombok.Getter;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;

import java.util.function.Supplier;
import java.util.function.BinaryOperator;

//----------------------------------------------------------------------[Declaration]---------------------------------------------------------------------------//
/**
 * <h1>Aggregator</h1>
 * 
 * <p>
 * 
 * 
 * @author Cody Washington (@Jelatinone) 
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class Aggregator<Aggregates> {
  //-----------------------------------------------------------------------[Constants]--------------------------------------------------------------------------//
  Supplier<Aggregates> ORIGIN;
  BinaryOperator<Aggregates> AGGREGATOR;
  //------------------------------------------------------------------------[Fields]----------------------------------------------------------------------------//
  /**
   * -- GETTER --
   *  Provides the value used most recently by {@link #aggregate()}, which was used to calculate the value of {@link #getAggregated() aggregated}.
   *  But notably not {@link #acquire()} (as it skips this step) 
   * @return Most recently calculated value of retained supplied via source during {@link #aggregate() aggregation}
   */
  @Getter @NonFinal volatile Aggregates Retained;
  /**
   * -- GETTER --
   *  Provides the previously {@link #aggregate() aggregated} or {@link #acquire() acquired} value defined by the Aggregator operator.
   * @return Most recently calculated value of {@link #aggregate()}
   */
  @Getter @NonFinal volatile Aggregates Aggregated;
  /**
   * -- GETTER --
   *  Provides the initial (first) value supplied (either via the constructor or by default) that was used during the first call to {@link #aggregate()} or
   * {@link #acquire()} before {@link #getRetained() retained} was modified.
   * @return Value used during the first call to {@link #acquire()} or {@link #aggregate()}
   */
  @Getter @NonFinal volatile Aggregates Initial;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Aggregator Constructor.
   * @param Origin      Supplier from which to derive the value of each following expression result defined by the aggregator operator
   * @param Aggregator  Expression operator determine the value of each successive step 
   */
  public Aggregator(final Supplier<Aggregates> Origin, final BinaryOperator<Aggregates> Aggregator) {
    this(Origin,Aggregator,Origin.get());
  }  

  /**
   * Aggregator Constructor.
   * @param Origin      Supplier from which to derive the value of each following expression result defined by the aggregator operator
   * @param Aggregator  Expression operator determine the value of each successive step 
   * @param Initial     First value of the {@link #getRetained() retained} amount
   */
  public Aggregator(final Supplier<Aggregates> Origin, final BinaryOperator<Aggregates> Aggregator, final Aggregates Initial) {
    ORIGIN = Origin;
    AGGREGATOR = Aggregator;
    this.Initial = Retained = Initial;
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Aggregates (operates) and sets the value of {@link #getAggregated() aggregated} via the Aggregator operator by providing the first argument as the most
   * recent value of the origin supplier, and using the previous value as the second argument. But notably, unlike {@link #acquire()} mutates the value
   * of Retained such retained now has the value of origin after the aggregation call has been made.
   * @return the value of {@link #getAggregated() aggregated}
   */
  public synchronized Aggregates aggregate() {
    synchronized(this) {
      return (Aggregated = AGGREGATOR.apply(Retained, Retained = ORIGIN.get()));
    }
  }

  /**
   * Aggregates (operates) and sets the value of {@link #getAggregated() aggregated} via the Aggregator operator by providing the first argument as the most
   * recent value of the origin supplier, and using the previous value as the second argument. 
   * @return the value of {@link #getAggregated() aggregated}
   */
  public synchronized Aggregates acquire() {
    synchronized(this) {
      return (Aggregated = AGGREGATOR.apply(Retained, ORIGIN.get()));
    }
  }

  /**
   * Mutates the current value of retained to a new value
   * @param Retained desired value of {@link #getRetained() retained}
   */
  public synchronized void retain(final Aggregates Retained) {
    synchronized(this) {
      this.Retained = Retained;
    }
  }

  /**
   * Mutates the currently stored value of retained and initial to a desired value; meaning that the value first supplied at the 
   * constructor is overwritten
   * @param Initial Value the {@link #getRetained() retained} and {@link #getInitial()} values should be modified to
   */
  public synchronized void reset(final Aggregates Initial) {
    synchronized(this) {
      this.Retained = this.Initial = Initial;
    }
  }
}
