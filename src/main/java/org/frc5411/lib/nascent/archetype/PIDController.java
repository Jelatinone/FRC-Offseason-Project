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
package org.frc5411.lib.nascent.archetype;
//-------------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//
import org.frc5411.lib.external.LoggedTunableNumber;
import org.frc5411.lib.nascent.Controller;
import org.frc5411.lib.pattern.Report;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import lombok.AccessLevel;
import lombok.Builder;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;
//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 * <h1>Controller</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class PIDController implements Controller<N2,N1,N1> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  LoggedTunableNumber PROPORTIONAL_GAIN, INTEGRAL_GAIN, DERIVATIVE_GAIN;
  edu.wpi.first.math.controller.PIDController CONTROLLER;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile @NonFinal Double Effort = (Double.NaN);
  volatile @NonFinal Double Timestamp = (Double.NaN);
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * PID Controller Constructor
   * @param Controller Standard PIDController object, pre-configured, which has the appropriate values to control a given mechanism.
   */
  public PIDController(final edu.wpi.first.math.controller.PIDController Controller) {
    CONTROLLER = Controller;
    PROPORTIONAL_GAIN = new LoggedTunableNumber(
      String.format(
        ("%s/Proportional"), 
        getIdentity()), 
      CONTROLLER.getP());
    INTEGRAL_GAIN = new LoggedTunableNumber(
      String.format(
        ("%s/Integral"), 
        getIdentity()), 
      CONTROLLER.getI());
    DERIVATIVE_GAIN = new LoggedTunableNumber(
      String.format(
        ("%s/Derivative"), 
        getIdentity()), 
      CONTROLLER.getD());
  }  

  /**
   * PID Controller Constructor
   * @param Descriptor Container of the relevant constants for a PID controller to be created, un-configure.
   */
  public PIDController(final Descriptor Descriptor) {
    this(new edu.wpi.first.math.controller.PIDController(
      Descriptor.Proportional, 
      Descriptor.Integral, 
      Descriptor.Derivative));
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized Vector<@NonNull N1> calculate(Vector<@NonNull N2> Reference) {
    synchronized(CONTROLLER) {
      Timestamp = HALUtil.getFPGATime() / 1e6;
      return VecBuilder
        .fill(Effort = CONTROLLER.calculate(
            Reference.get(
              (0), 
              (0)), 
            Reference.get(
              (1), 
              (0))
          )
        );      
    }
  }

  /**
   * Enables continuous input by considering the minimum and maximum bounds to be the same point, allowing the shortest route
   * between them to be calculated
   * @param Bounds a vector containing two elements, the first is the lower bound, and the second the upper bound
   * @see #discontinuous()
   */
  @Override
  public synchronized void continuous(final Vector<N2> Bounds) {
    CONTROLLER.enableContinuousInput(
      Bounds.get(
        (0),
        (0)), 
      Bounds.get(
        (1),
        (0))
    );
  }

  /**
   * Disables continuous input, which allows for the shortest route between two points, a minimum and maximum bound, to be found by
   * considering them the same point.
   * @see #continuous(Vector)
   */
  @Override
  public synchronized void discontinuous() {
    synchronized(CONTROLLER) {
      CONTROLLER
        .disableContinuousInput();
    }
    
  }

  @Override
  public synchronized void reset() {
    synchronized(CONTROLLER) {
      CONTROLLER
        .reset();
    }
  }

  @Override
  public final PIDController clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(
      String.format(
        ("%s Instances Cannot Be Cloned"), 
        getClass()
          .getSimpleName()));
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  @Override
  public Vector<@NonNull N2> getStates() {
    return VecBuilder.fill(
      CONTROLLER
        .getSetpoint(), 
      getOutputs()
        .get((0), (0))
    );
  }

  @Override
  public Vector<@NonNull N1> getInputs() {
    return VecBuilder.fill(Effort);
  }

  @Override
  public Vector<@NonNull N1> getOutputs() {
    return VecBuilder.fill(
      CONTROLLER
        .getSetpoint() 
            + 
      CONTROLLER
        .getPositionError());
  }

  @Override
  public Vector<@NonNull N1> getError() {
    return VecBuilder.fill(
      CONTROLLER.getPositionError()
    );
  }

  @SuppressWarnings("unchecked")
  @Override
  public void update(@NonNull Report<org.frc5411.lib.utility.Vector<Double,N1>> Record) {
    synchronized(CONTROLLER) {
      CONTROLLER
        .setP(PROPORTIONAL_GAIN.getAsDouble());
      CONTROLLER
        .setI(INTEGRAL_GAIN.getAsDouble());
      CONTROLLER
        .setD(DERIVATIVE_GAIN.getAsDouble());      
    }
    Record.setConnected((true));
    Record.setTimestamps(
      new double[] {Timestamp}
    );
    Record.setMeasurements(
      new org.frc5411.lib.utility.Vector[] {new org.frc5411.lib.utility.Vector<>(() -> (1), Effort)}
    );
  }
  //----------------------------------------------------------------------[Internal]---------------------------------------------------------------------------//
  /**
   * <Descriptor>
   * 
   * 
   */
  @Builder(toBuilder = (true))
  @FieldDefaults(level = AccessLevel.PUBLIC, makeFinal = (true))
  public static final class Descriptor extends org.frc5411.lib.pattern.Descriptor<PIDController> {
    Double Proportional, Integral, Derivative;

    @Override
    public Descriptor clone() {
      return new Descriptor(
        Proportional,
        Integral,
        Derivative);
    }
  }
}
