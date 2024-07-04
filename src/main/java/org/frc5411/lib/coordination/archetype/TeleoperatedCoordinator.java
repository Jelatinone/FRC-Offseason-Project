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
import org.frc5411.lib.instrument.module.Limit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

import java.util.Objects;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;
//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 * <h1>TeleoperatedCoordinator</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class TeleoperatedCoordinator implements Coordinator<Twist2d> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Double DEADBAND;
  Limit LIMITS;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile @NonFinal Twist2d Setpoint;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Teleoperated Coordinator Constructor.
   * @param Deadband  Limit at which all the absolute value of all inputs less than will be rounded to zero 
   * @param Chassis   Physical movement limits of the chassis in two-dimensional space 
   */
  public TeleoperatedCoordinator(final Double Deadband, final Limit Chassis) {
    DEADBAND = Deadband;
    LIMITS = Objects
      .requireNonNull(Chassis);
    Setpoint = new Twist2d();
  }

  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Updates the value to coordinate against, i.e. the value of the controller's setpoint
   * @param Effort Setpoint of internal controller
   */
  public synchronized void coordinate(final Twist2d Effort) {
    synchronized(Effort) {
      this.Setpoint = Objects
        .requireNonNull(Effort);
    }
  }

  @Override
  public synchronized Twist2d update() {
    synchronized(Setpoint) {
      final var Velocity = calculate(Setpoint.dx, Setpoint.dy);
      final var Omega = MathUtil.applyDeadband(Setpoint.dtheta, DEADBAND);
      return new Twist2d(
        Velocity
          .getX() 
            * 
        LIMITS
          .TranslationalVelocity(), 
        Velocity
          .getY() 
            * 
        LIMITS
          .TranslationalVelocity(), 
        Math
          .copySign(Omega * Omega, Omega) 
            * 
        LIMITS
          .RotationalVelocity()
      );
    }
  }

  /**
   * Calculates the desired vector of translation for a given {@code X} and {@code Y} input
   * @param x Scalar value along the x-axis (horizontal) in two-dimensional space; maybe-negative.
   * @param y Scalar value along the Y-axis (vertical) in two-dimensional space; maybe-negative.
   * @return Vector ({@link Translation2d}) of the desired state; placed at the origin
   */
  private Translation2d calculate(final Double x, final Double y) {
    var Direction = new Rotation2d(x, y);
    var Magnitude = Math.pow(
      MathUtil
        .applyDeadband(Math.hypot(x, y), DEADBAND), 
      (2D)
    );
    return new Pose2d(new Translation2d(), Direction)
      .transformBy(new Transform2d(
        Magnitude, // <--- Something seems a little off here?
        (0D), 
        new Rotation2d()))
      .getTranslation();
  }
  
}
