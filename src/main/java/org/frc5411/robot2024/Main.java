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
package org.frc5411.robot2024;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 *
 *
 * <h1>Main</h1>
 *
 * <p>Robot Project runner class, responsible for robot initialization by starting the Driverstation, CameraServer, and HAL services.
 *
 * @see Robot
 */
public final class Main {
  //----------------------------------------------------------------------[Methods]----------------------------------------------------------------------------//
  /**
   * Initializes the robot and underlying systems
   * @param Options Additional options applied via the command line
   */
  public static synchronized void main(final String... Options) {
    RobotBase.startRobot(Robot::getInstance);
  }
}