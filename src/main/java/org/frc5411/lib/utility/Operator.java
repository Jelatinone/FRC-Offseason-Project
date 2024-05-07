//------------------------------------------------------------------------[Package]------------------------=----------------------------------------------------//
package org.frc5411.lib.utility;
//-----------------------------------------------------------------------[Libraries]----------------------------------------------------------------------------//
import java.util.function.Supplier;
import java.util.function.Consumer;
import java.util.function.BinaryOperator;
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
  //------------------------------------------------------------------------[Fields]----------------------------------------------------------------------------//
  private volatile Type Retained;
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
   * shorthand for {@link #accept(double)} where {@link #getInitial()} is the argument
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
   * Provides the previously saved value of this operator directly from the source
   * @return Retained value based on the value of it's source
   */
  public Type getRetained() {
    return Retained;
  }

  /**
   * Provides the initial value either passed in as an argument or supplied from the source function
   * @return Initial value based on source or argument
   */
  public Type getInitial() {
    return INITIAL;
  }

  /**
   * Provides the previously operated value of this operator
   * @return Operated value based on the evaluation of it's expression
   */
  public Type getOperated() {
    return Operated;
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
