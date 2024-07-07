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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

import static org.frc5411.lib.utility.Figures.*;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Geometry</h1>
 * 
 * <p>Utility class for anything relating to geometry, houses a handful of static methods which perform simple, but nonetheless repetitive calculations.
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class Geometry {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//

  public static Pose2d exp(final Twist2d Delta) {
      final double Sin = Math.sin(Delta.dtheta);
      final double Cos = Math.cos(Delta.dtheta);
      double s, c;
      if (Math.abs(Delta.dtheta) < EQUIVALENCE) {
          s = 1.0 - 1.0 / 6.0 * Delta.dtheta * Delta.dtheta;
          c = .5 * Delta.dtheta;
      } else {
          s = Sin / Delta.dtheta;
          c = (1.0 - Cos) / Delta.dtheta;
      }
      return new Pose2d(new Translation2d(Delta.dx * s - Delta.dy * c, Delta.dx * c + Delta.dy * s),
              new Rotation2d(Cos, Sin));
  }


  public static Twist2d log(final Pose2d Transform) {
      final double Theta = Transform.getRotation().getRadians();
      final double Half = Theta / 2D;
      final double Cosine = Transform.getRotation().getCos() - 1D;
      final double Tangent = (Math.abs(Cosine) < EQUIVALENCE)?  1D - 1D / 12D * Theta * Theta:  -(Half * Transform.getRotation().getSin()) / Cosine;
      final Translation2d translation_part = Transform.getTranslation()
              .rotateBy(new Rotation2d(Tangent, -Half));
      return new Twist2d(translation_part.getX(), translation_part.getY(), Theta);
  }
}
