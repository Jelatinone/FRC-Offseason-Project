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
import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N0;

import com.jcabi.aspects.Immutable.Array;

import java.util.Objects;
import javax.validation.constraints.NotNull;

//----------------------------------------------------------------------[Declaration]---------------------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Vector</h1>
 * 
 * <p>Simple utility wrapper class for an immutable generic array of elements with a pre-defined size.<p>
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class Vector<@NotNull Type, @NotNull Elements extends Num> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  private static final Vector<Object,N0> EMPTY = new Vector<>(Nat.N0());
  private final @NotNull @Array Type[] VECTOR;

  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  /**
   * Type Vector Constructor.
   * @param Elements              Natural number representation of the number of elements in this array
   * @param Vector                Array data to place within the bounds of the Vector's array, length should match the number of elements specified.
   * @throws BoundaryException    Bounds of the array are exceeded or not met by the length of the Vector parameter
   * @throws NullPointerException Either the Elements parameter or the Vector parameter evaluate to null 
   */
  @SuppressWarnings("unchecked")
  public Vector(final Nat<Elements> Elements, final Type... Vector) throws BoundaryException {
    try {
      assert Elements.getNum() == Objects.requireNonNull(Vector).length;
    } catch(final AssertionError Ignored) {
      throw new BoundaryException(
        String.format(
          ("Bounds of Vector defined, [%d], do not match the length of the vector, [%d], excepted"), 
          Elements.getNum(), 
          Vector.length));
    }
    VECTOR = Objects.requireNonNull(Vector);
  }
  //------------------------------------------------------------------------[Methods]-------------------------------------------------------------------------//
  /**
   * Creates a type vector with the specified elements
   * @param <Type>                Type of the generic array
   * @param Vector                Array data to place within the bounds of the Vector's array, length should match the number of elements specified.
   * @throws BoundaryException    Bounds of the array are exceeded or not met by the length of the Vector parameter
   * @throws NullPointerException Either the Elements parameter or the Vector parameter evaluate to null 
   * @return Vector of the given elements
   */
  @SuppressWarnings("unchecked")
  public static <Type, Elements extends Num> Vector<Type,Elements> fill(final Type... Vector) {
    return new Vector<>(() -> Vector.length, Vector);
  }

  /**
   * Creates an empty type vector, where there are no elements within the underlying generic array
   * @param <Type> Type of the generic array
   * @return An array consisting of zero elements, with a specified type.
   */
  @SuppressWarnings("unchecked")
  public static <Type, Elements extends Num> Vector<Type, Elements> empty() {
    return (Vector<Type, Elements>) EMPTY;
  }
  //-----------------------------------------------------------------------[Accessors]------------------------------------------------------------------------//
  /**
   * Provides the underlying, specified type, array that was defined during construction, will always meet the expected bounds of Elements.
   * @return Array of specified type
   * @see #get(Integer)
   */
  public Type[] getArray() {
    return VECTOR;
  }

  /**
   * Provides the value of a specific point within the array
   * @param Index Point within the array to get a value from
   * @see #getArray()
   */
  public synchronized Type get(final Integer Index) {
    return VECTOR[Index];
  }  
}
