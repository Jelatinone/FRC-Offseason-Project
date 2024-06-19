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

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;

import com.jcabi.aspects.Immutable.Array;

import java.util.Objects;
import java.util.stream.Stream;
import java.util.Collection;

import java.util.function.Consumer;
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
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class Vector<@NonNull Type, @NonNull Elements extends Num> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  static Vector<Object,N0> EMPTY = new Vector<>(Nat.N0(), new Object[0]);
  
  @Array Type[] VECTOR;
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
   * Creates a type vector with the specified elements
   * @param <Type>                Type of the collection
   * @param Vector                Collection data to place within the bounds of the Vector's collection, {@link Collection#size() size} should match the number 
   *                              of elements specified.
   * @throws BoundaryException    Bounds of the collection are exceeded or not met by the length of the Vector parameter
   * @throws NullPointerException Either the Elements parameter or the Vector parameter evaluate to null 
   * @return Vector of the given elements
   */
  @SuppressWarnings("unchecked")
  public static <Type, Elements extends Num> Vector<Type,Elements> fill(final Collection<Type> Vector) {
    return new Vector<>(() -> Vector.size(), (Type[]) Vector.toArray());
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

  /**
   * Returns a sequential {@code Stream} with this collection as its source.
   *
   * @implSpec
   * The default implementation creates a sequential {@code Stream} from the
   * collection's {@code Spliterator}.
   *
   * @return a sequential {@code Stream} over the elements in this collection
   */
  public Stream<Type> stream() {
    return Stream.of(VECTOR);
  }

  /**
   * Performs the given action for each element of the {@code Iterable}
   * until all elements have been processed or the action throws an
   * exception.  Actions are performed in the order of iteration, if that
   * order is specified.  Exceptions thrown by the action are relayed to the
   * caller.
   * <p>
   * The behavior of this method is unspecified if the action performs
   * side-effects that modify the underlying source of elements, unless an
   * overriding class has specified a concurrent modification policy.
   *
   * @implSpec
   * <p>The default implementation behaves as if:
   * <pre>{@code
   *     for (T t : this)
   *         action.accept(t);
   * }</pre>
   *
   * @param Action Consumer to be applied against each element stored within the vector
   * @throws NullPointerException If the specified action is null
   */
  public void forEach(final Consumer<? super Type> Action) {
    Objects.requireNonNull(Action);
    for(final var Element: VECTOR) {
      Action.accept(Element);
    }
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
}
