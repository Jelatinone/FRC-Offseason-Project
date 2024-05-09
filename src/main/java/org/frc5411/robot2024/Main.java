//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.robot2024;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
//----------------------------------------------------------------------[Declaration]----------------------------------------------------------------------//
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
  //----------------------------------------------------------------------[Methods]------------------------------------------------------------------------//
  /**
   * Initializes the robot and underlying systems
   * @param Options Additional options applied via the command line
   */
  public static synchronized void main(final String... Options) {
    RobotBase.startRobot(Robot::getInstance);
  }
}