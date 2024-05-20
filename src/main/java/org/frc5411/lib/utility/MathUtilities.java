//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.utility;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * 
 * 
 * <h1>MathUtilities</h1>
 * 
 * <p>Simple math utility helper functionality, provides simple static methods for doing simple, but repetitive calculations; add methods as needed.
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class MathUtilities {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Provides the standard deviation for a given set of numbers 
   * @param Numbers Collection (array) of data to find the standard deviation of
   * @return Standard deviation as a double value
   */
  public static double standardDeviation(final double... Numbers){
    double Mean = mean(Numbers), SummativeSquareDifference = 0d;
    for(double Number : Numbers){
      SummativeSquareDifference += Math.pow((Number - Mean), (2));
    }
    return Math.sqrt(SummativeSquareDifference / Numbers.length - 1);
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
    double Mean = 0d;
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