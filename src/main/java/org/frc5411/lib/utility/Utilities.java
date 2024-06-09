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
//-----------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.util.DoubleCircularBuffer;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Utilities</h1>
 * 
 * <p>Simple math utility helper functionality, provides simple static methods for doing simple, but repetitive calculations; add methods as needed.
 * 
 * @author Cody Washington (@Jelatinone) 
 */
@FieldDefaults(level = AccessLevel.PUBLIC, makeFinal = (true))
public class Utilities {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679D;
  Double E = 2.7182818284590452353602874713526624977572470936999595749669676277240766303535475945713821785251664274D;
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Provides the standard deviation for a given set of numbers 
   * @param Numbers Collection (array) of data to find the standard deviation of
   * @return Standard deviation as a double value
   */
  public static double standardDeviation(final double... Numbers){
    double Mean = mean(Numbers), Sum = (0d);
    for(final double Number : Numbers){
      Sum += Math.pow((Number - Mean), (2));
    }
    return Math.sqrt(Sum / Numbers.length - 1);
  }

  /**
   * Transforms a circular buffer into an array of equivalent length and elements in the same order, but is not destructive to the original buffer. 
   * i.e. the original elements of the buffer are retained.
   * @param Source Buffer source to accept elements from into the new array
   * @return Array with the same elements, in the same order
   */
  static double[] from(final DoubleCircularBuffer Source) {
    final var Size = Source.size();
    final var Array = new double[Size];
    for(int Index = (0); Index < Size; Index++) {
      Array[Index] = Source.get(Index);
    }
    return Array;
  }

  /**
   * Transforms a circular array into a buffer of equivalent length and elements in the same order, but is not destructive to the original array.
   * i.e. the original elements of the array are retained.
   * @param Source Array source to accept elements from into the new buffer
   * @return Buffer with the same elements, in the same order
   */
  static DoubleCircularBuffer from(final double[] Source) {
    final var Buffer = new DoubleCircularBuffer(Source.length);
    for(final double Element: Source) {
      Buffer.addLast(Element);
    }
    return Buffer;
  }

  /**
   * Provides squared inputs to a given input, while retaining the sign
   * @param Input Any Real Number
   * @return Input Squared, with the same sign of the original
   */
  public static double squareSigned(final double Input) {
    return Math.copySign(Input * Input, Input);
  }

  /**
   * Provides the mean or 'simple average' for a set of numbers
   * @param Numbers Collection (array) of data to find the standard deviation of
   * @return Mean as a double value
   */
  public static double mean(final double... Numbers) {
    double Mean = (0d);
    for(double Number : Numbers){
      Mean += Number;
    }
    Mean /= Numbers.length;
    return Mean;
  }

  /**
   * Shorthand for unwrapping a single 1 x 1 matrix into a single double element of equivalent values
   * @param Matrix Single-row Single-column matrix to be unwrapped
   * @return Unwrapped value
   */
  public static double unwrap(final Matrix<N1,N1> Matrix) {
    return Matrix.get((0), (0));
  }

  /**
   * Shorthand for wrapping a vector, essentially a one-dimensional matrix from a list of double elements
   * @param <Elements> Number of elements within the array itself as a functional interface
   * @param Elements Elements to be placed into the new Matrix
   * @return Matrix, filled with elements in the same order they were received.
   */
  public static <Elements extends Num> Matrix<Elements,N1> wrap(final double... Elements) {
    return MatBuilder.fill(() -> Elements.length, Nat.N1(), Elements);
  }
}