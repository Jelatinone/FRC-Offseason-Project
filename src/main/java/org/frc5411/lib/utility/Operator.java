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
//--------------------------------------------------------------------------[Package]--------------------------------------------------------------------------//
package org.frc5411.lib.utility;
//-----------------------------------------------------------------------[Libraries]----------------------------------------------------------------------------//
import java.util.function.BinaryOperator;
import java.util.function.Consumer;
import java.util.function.Supplier;

import lombok.Getter;
//----------------------------------------------------------------------[Declaration]---------------------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Difference</h1>
 * 
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class Operator<Type extends Number> implements Supplier<Type>, Consumer<Type> {
  //-----------------------------------------------------------------------[Constants]--------------------------------------------------------------------------//
  private final Supplier<Type> SOURCE;
  private final BinaryOperator<Type> OPERATOR;
  private final Type INITIAL;
  /**
   * -- GETTER --
   *  Provides the previously saved value of this operator directly from the source
   *
   * @return Retained value based on the value of it's source
   */
  //------------------------------------------------------------------------[Fields]----------------------------------------------------------------------------//
  @Getter
  private volatile Type Retained;
  /**
   * -- GETTER --
   *  Provides the previously operated value of this operator
   *
   * @return Operated value based on the evaluation of it's expression
   */
  @Getter
  private volatile Type Operated;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Operator Constructor.
   * @param Source    Supplier from which to derive the value of each following expression from
   * @param Operator  Expression determining the value of each following step
   */
  public Operator(final Supplier<Type> Source, final BinaryOperator<Type> Operator) {
    OPERATOR = Operator;
    SOURCE = Source;
    INITIAL = Retained = SOURCE.get();
  }

  /**
   * Operator Constructor.
   * @param Source    Supplier from which to derive the value of each following expression from
   * @param Operator  Expression determining the value of each following step
   * @param Initial   First value to begin the following expressions with
   */
  public Operator(final Supplier<Type> Source, final BinaryOperator<Type> Operator, final Type Initial) {
    OPERATOR = Operator;
    SOURCE = Source;
    INITIAL = Retained = Initial;
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//

  /**
   * Mutates the retained value to the initial value passed as an argument or supplied from the source
   * shorthand for {@link #accept(Type)} where {@link #getInitial()} is the argument
   */
  public synchronized void reset() {
    synchronized(Retained) {
      Retained = INITIAL;
    }
  }
  /**
   * Mutates the retained value to a specified number of the same type
   * @param Retained Saved value from source, being directly modified
   */
  public synchronized void accept(final Type Retained) {
    synchronized(this.Retained) {
      this.Retained = Retained;
    }
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//

  /**
   * Provides the initial value either passed in as an argument or supplied from the source function
   * @return Initial value based on source or argument
   */
  public Type getInitial() {
    return INITIAL;
  }


  /**
   * Provides the operated value of the source (current) and the previous value
   * @return Operated value based on the evaluation of it's expression
   */
  public synchronized Type get() {
    synchronized(Retained) {
      return (Operated = OPERATOR.apply(Retained, Retained = SOURCE.get()));
    }
  }
}
