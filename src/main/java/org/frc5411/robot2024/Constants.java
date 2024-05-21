//------------------------------------------------------------------------[Package]----------------------------------------------------------------------------//
package org.frc5411.robot2024;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
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
  //----------------------------------------------------------------------[Methods]----------------------------------------------------------------------------//
  /**
   * Performs a pre-deployment check for Deployment of the robot, ensuring that the robot is running
   * on real-hardware, with the correct mode selected
   * @param Options Additional options applied via the command line
   * 
   */
  public static synchronized void main(final String... Options) {
    if(Robot.TYPE == Type.SIMBOT) {
      System.exit((1));
    }
  }
  //----------------------------------------------------------------------[Internal]---------------------------------------------------------------------------//
  public static final class Robot {
    private static final Type DESIRED_TYPE = Type.DEVBOT;
    public static final Type TYPE = RobotBase.isReal()? DESIRED_TYPE: Type.SIMBOT;
    public static final Mode MODE = switch(TYPE) {
      case DEVBOT, COMPBOT 
        -> RobotBase.isReal()? Mode.ACTUAL: Mode.REPLAY;
      case ANONBOT
        -> Mode.ANONYMOUS;
      case SIMBOT 
        -> Mode.SIMULATED;
    };
    public static final Profile DRIVER =  TYPE.equals(Type.COMPBOT)? Profile.COMP_DRIVER: Profile.DEV_DRIVER;
    public static final Profile OPERATOR = TYPE.equals(Type.COMPBOT)? Profile.COMP_OPERATOR: Profile.DEV_OPERATOR;
  }
}
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
/**
 * Represents the mode of the robot being initialized, i.e. whether we are running on real or simulated hardware, and if we are 
 * replaying from a logged source.
 */
enum Mode {

  ANONYMOUS,

  ACTUAL,

  SIMULATED,

  REPLAY,
}

/**
 * Represents the pre-set mode a robot is launched into, i.e. a setting to distinguish between the different stages of robot 
 * development for testing purposes.
 */
enum Type {

  ANONBOT,

  DEVBOT,

  SIMBOT,

  COMPBOT,
}

/**
 * Represents a different pre-set profile for different drivers operating the robot, i. e, drivers with different preferences for keybindings
 * and robot operation
 */
enum Profile {
  
  DEV_DRIVER,

  DEV_OPERATOR,

  COMP_DRIVER,

  COMP_OPERATOR,
}