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
package org.frc5411.lib.coordination.archetype;
//-------------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//
import org.frc5411.lib.coordination.Coordinator;
import org.frc5411.lib.nascent.Controller;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import java.util.function.Supplier;
import java.util.Objects;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;

import static org.frc5411.lib.utility.Figures.*;
//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 * <h1>HeadingCoordinator</h1>
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class HeadingCoordinator implements Coordinator<Double> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Controller<N2,N1,N1> CONTROLLER;
  Supplier<Rotation2d> OBSERVER;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile @NonFinal Rotation2d Setpoint;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Heading Coordinator Constructor
   * @param Controller Properly configured controller of heading states, which drives the error between setpoint and measurement to zero.
   * @param Observer   Chassis heading state observer; always reports the most recent stable measurement of heading
   */
  public HeadingCoordinator(final Controller<N2,N1,N1> Controller, final Supplier<Rotation2d> Observer) {
    CONTROLLER = Objects
      .requireNonNull(Controller);
    OBSERVER = Objects
      .requireNonNull(Observer);
    CONTROLLER
      .reset();
    Setpoint = new Rotation2d();
  } 
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Updates the value to coordinate against, i.e. the value of the controller's setpoint
   * @param Effort Setpoint of internal controller
   */
  public synchronized void coordinate(final Rotation2d Effort) {
    synchronized(Effort) {
      this.Setpoint = Objects
        .requireNonNull(Effort);
    }
  }

  @Override
  public synchronized Double update() {
    return unwrap(CONTROLLER.calculate(
      VecBuilder.fill(
        OBSERVER
          .get()
          .getRadians(), 
        Setpoint
          .getRadians()
      )
    ));
  }
}
