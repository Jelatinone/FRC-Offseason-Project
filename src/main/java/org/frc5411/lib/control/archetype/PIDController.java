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
import org.frc5411.lib.control.Controller;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import lombok.NonNull;
//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 * <h1>PIDController</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public class PIDController extends edu.wpi.first.math.controller.PIDController implements Controller<N2,N1,N1> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  private volatile Double Effort = (0d);
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * PID Controller Constructor
   * @param Constants Container object of Relevant PID tuned constants: P(roptional), I(ntegral), D(erivative) 
   */
  public PIDController(final PIDConstants Constants) {
    super(Constants.getProportional(), Constants.getIntegral(), Constants.getDerivative());
  }

  /**
   * PID Controller Constructor
   * @param Constants Standard PIDController object with Relevant PID tuned constants: P(roptional), I(ntegral), D(erivative) 
   */
  public PIDController(final edu.wpi.first.math.controller.PIDController Controller) {
    super(Controller.getP(), Controller.getI(), Controller.getD());
  }  
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized Matrix<@NonNull N1, N1> calculate(Matrix<@NonNull N2, N1> Reference) {
    return MatBuilder.fill(Nat.N1(), Nat.N1(), Effort = calculate(Reference.get((0), (0)), Reference.get((0), (1))));
  }

  /**
   * Rather than interpreting the minimum and maximum as separate points, a 'roll-over' is performed and the shortest route can be calculated between them by treating
   * them as equivalent.
   * @param Minimum Minimum input value for which a roll-over occurs
   * @param Maximum Maximum input value for which a roll-over occurs
   */
  public void continuous(final Double Minimum, final Double Maximum) {
    super.enableContinuousInput(Minimum, Maximum);
  }

  /**
   * Disables {@link #continuous(Double, Double) continuous} input, disregarding the system's minimum and maximum bounds and treating them as separate points that
   * are not equivalent.
   */
  public void discrete() {
    super.disableContinuousInput();
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  @Override
  public Matrix<@NonNull N2, N1> getStates() {
    final var Setpoint = getSetpoint();
    return MatBuilder.fill(Nat.N2(), Nat.N1(), (Setpoint - getPositionError()) ,Setpoint);
  }

  @Override
  public Matrix<@NonNull N1, N1> getInputs() {
    return MatBuilder.fill(Nat.N1(), Nat.N1(), Effort);
  }

  @Override
  public Matrix<@NonNull N1, N1> getError() {
    return MatBuilder.fill(Nat.N1(), Nat.N1(), getPositionError());
  }
  
}
