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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.Supplier;

import org.frc5411.lib.utility.Profile;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
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
@FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
public final class Constants {
  //----------------------------------------------------------------------[Methods]----------------------------------------------------------------------------//
  /**
   * Performs a pre-deployment check for Deployment of the robot, ensuring that the robot is running
   * on real-hardware, with the correct mode selected
   * @param Options Additional options applied via the command line
   * 
   */
  public static synchronized void main(final String... Options) {
    if(Identity.TYPE == Type.SIMBOT) {
      System.exit((1));
    }
  }
  //----------------------------------------------------------------------[Internal]---------------------------------------------------------------------------//
  @FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
  public static final class Identity {
    static Type DESIRED_TYPE = Type.SIMBOT;
    static Type TYPE = RobotBase.isReal()? DESIRED_TYPE: Type.SIMBOT;
    static Mode MODE = switch(TYPE) {
      case DEVBOT, COMPBOT 
        -> RobotBase.isReal()? Mode.ACTUAL: Mode.REPLAY;
      case ANONBOT
        -> Mode.ANONYMOUS;
      case SIMBOT 
        -> Mode.SIMULATED;
    };

    static Boolean DRIVEBASE_ENABLED = (true);
    static Boolean VISION_ENABLED = (false);

    static Integer DRIVER_CONTROL_PORT = (0);
    static CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(DRIVER_CONTROL_PORT);

    static Integer OPERATOR_CONTROL_PORT = (1);
    static CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(OPERATOR_CONTROL_PORT);

    static Profile<Keybindings,Preferences> DRIVER =  (TYPE.equals(Type.COMPBOT)? Character.COMP_DRIVER: Character.DEV_DRIVER).get();
    static Profile<Keybindings,Preferences> OPERATOR = (TYPE.equals(Type.COMPBOT)? Character.COMP_OPERATOR: Character.DEV_OPERATOR).get();

    static AprilTagFields FIELD = AprilTagFields.kDefaultField;
  }
  //-----------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   * <h1>Mode</h1>
   * 
   * <p>Represents the mode of the robot being initialized, i.e. whether we are running on real or simulated hardware, and if we are
   * replaying from a logged source.
   */
  public enum Mode {

    ANONYMOUS,

    ACTUAL,

    SIMULATED,

    REPLAY,
  }

  /**
   * <h1>Type</h1>
   * 
   * <p>Represents the pre-set mode a robot is launched into, i.e. a setting to distinguish between the different stages of robot
   * development for testing purposes.
   */
  public enum Type {

    ANONBOT,

    DEVBOT,

    SIMBOT,

    COMPBOT,
  }

  /**
   * <h1>Character</h1>
   * 
   * <p>Represents a different pre-set profile for different drivers operating the robot, i. e, drivers with different preferences for keybindings
   * and robot operation.
   */
  @SuppressWarnings("resource")
  public enum Character implements Supplier<Profile<Keybindings,Preferences>> {

    DEV_DRIVER(
      new org.frc5411.lib.utility.Profile<Keybindings,Preferences>(("DEV_DRIVER"))
        .add(Preferences.CONTROL_EFFORT_X, (Supplier<Double>) () -> Identity.DRIVER_CONTROLLER.getRawAxis((1)))
        .add(Preferences.CONTROL_ZONE_X, (2e-1D))
        .add(Preferences.CONTROL_EFFORT_Y, (Supplier<Double>) () -> Identity.DRIVER_CONTROLLER.getRawAxis((0)))
        .add(Preferences.CONTROL_ZONE_Y, (2e-1D))
        .add(Preferences.CONTROL_EFFORT_T, (Supplier<Double>) () -> Identity.DRIVER_CONTROLLER.getRawAxis((4)))
        .add(Preferences.CONTROL_ZONE_T, (2e-1D))
    ),

    DEV_OPERATOR(
      new Profile<Keybindings,Preferences>(("DEV_OPERATOR"))
    ),

    COMP_DRIVER(
      new Profile<Keybindings,Preferences>(("COMP_DRIVER"))
    ),

    COMP_OPERATOR(
      new Profile<Keybindings,Preferences>(("COMP_OPERATOR"))
    );

    private final Profile<Keybindings,Preferences> PROFILE;

    /**
     * Profile Constructor
     * @param Profile Individual's profile with selected preferences and keybindings which act as settings for different robot functionality
     */
    Character(final Profile<Keybindings,Preferences> Profile) {
      PROFILE = Profile;
    }

    /**
     * Provides the retained operator profile (with settings and keybindings) as settings
     * @return Retained profile instance
     */
    @Override
    public final Profile<Keybindings,Preferences> get() {
      return PROFILE;
    }
  }

  /**
   * <h1>Keybindings</h1>
   */
  public enum Keybindings {
    STATE_TOGGLE,    
    GYROSCOPE_RESET,
  }

  /**
   * <h1>Preferences</h1>
   */
  public enum Preferences {
    CONTROL_EFFORT_X,
    CONTROL_ZONE_X,
    CONTROL_EFFORT_Y,
    CONTROL_ZONE_Y,
    CONTROL_EFFORT_T,
    CONTROL_ZONE_T,
    CONTROL_SQUARED,
  }  
}