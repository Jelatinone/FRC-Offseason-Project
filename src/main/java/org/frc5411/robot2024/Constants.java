//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.robot2024;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//

import edu.wpi.first.wpilibj.RobotBase;

//----------------------------------------------------------------------[Declaration]----------------------------------------------------------------------//
/**
 *
 *
 * <h1>RobotConstants</h1>
 *
 * <p>Contains all robot-wide constants, does not contain subsystem specific constants.
 *
 * @see Manager
 */
public final class Constants {
  //----------------------------------------------------------------------[Methods]------------------------------------------------------------------------//
  /**
   * Initializes the robot and underlying systems
   * @param Options Additional options applied via the command line
   */
  public static synchronized final void main(final String... Options) {

  }
  //----------------------------------------------------------------------[Internal]-----------------------------------------------------------------------//
  public static final class Robot {
    public static final Type TYPE = Type.COMPBOT;
    public static final Mode MODE = switch(TYPE) {
      case DEVBOT, COMPBOT 
        -> RobotBase.isReal()? Mode.ACTUAL: Mode.REPLAY;
      case SIMBOT 
        -> Mode.SIMULATED;
    };
  }
}
//-----------------------------------------------------------------------[External]------------------------------------------------------------------------//
/**
 * Represents the mode of the robot being initialized, i.e. whether we are running on real or simulated hardware, and if we are 
 * replaying from a logged source.
 */
enum Mode {

  ACTUAL,

  SIMULATED,

  REPLAY,
}

/**
 * Represents the pre-set mode a robot is launched into, i.e. a setting to distinguish between the different stages of robot 
 * development for testing purposes.
 */
enum Type {

  DEVBOT,

  SIMBOT,

  COMPBOT,
}