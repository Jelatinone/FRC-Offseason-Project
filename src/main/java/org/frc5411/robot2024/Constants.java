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
import org.frc5411.lib.utility.Profile;

import org.frc5411.robot2024.subsystems.drivebase.DrivebaseSubsystem;
import org.frc5411.robot2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.List;
import java.util.function.Supplier;

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
   */
  public static synchronized void main(final String... Options) {
    if(Identity.DESIRED_TYPE.equals(Type.SIMBOT)) {
      System.exit((1));
    }
  }
  //----------------------------------------------------------------------[Internal]---------------------------------------------------------------------------//
  @FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
  public static final class Identity {
    static Integer THREAD_PARALLELISM = (8);
    static Integer UPDATE_FREQUENCY = (100);

    static Integer QUEUE_SIZE = (50);
    static Double BUFFER_SIZE = (2D);
    static Double MAXIMUM_CORRECTION = (2D);

    static List<Supplier<Subsystem>> MANAGEABLE = List.of(
      VisionSubsystem::getInstance,
      DrivebaseSubsystem::getInstance
    ); 

    static Type DESIRED_TYPE = Type.DEVBOT; 
    static Type TYPE = RobotBase.isReal()? DESIRED_TYPE: Type.SIMBOT;
    static Mode MODE = switch(TYPE) {
      case DEVBOT, COMPBOT
        -> RobotBase.isReal()? Mode.ACTUAL: Mode.REPLAY;
      case ANONBOT
        -> Mode.ANONYMOUS;
      case SIMBOT 
        -> Mode.SIMULATED;
    };

    static AprilTagFields FIELD = AprilTagFields.kDefaultField;
  }

  @FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
  public static final class Field {
    static Double LENGTH = (16.54106D);
    static Double WIDTH = (8.211236D);

    static Double MARGIN = (5E-1D);
    static Double ELEVATION = (7.5E-1D);
  }

  @FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
  public static final class Control {
    static Integer DRIVER_CONTROL_PORT = (0);
    static Integer OPERATOR_CONTROL_PORT = (1);

    static CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(DRIVER_CONTROL_PORT);
    static CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(OPERATOR_CONTROL_PORT);

    static Profile<Keybindings,Preferences> DRIVER =  
      (Identity.TYPE.equals(Type.COMPBOT)? 
        Profiles.COMP_DRIVER: 
        Profiles.DEV_DRIVER).get();
    static Profile<Keybindings,Preferences> OPERATOR = 
      (Identity.TYPE.equals(Type.COMPBOT)? 
        Profiles.COMP_OPERATOR: 
        Profiles.DEV_OPERATOR).get();
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
   * <h1>Profiles</h1>
   * 
   * <p>Represents a different pre-set profile for different drivers operating the robot, i. e, drivers with different preferences for keybindings
   * and robot operation.
   */
  @SuppressWarnings("resource")
  public enum Profiles implements Supplier<Profile<Keybindings,Preferences>> {

    DEV_DRIVER(
      new org.frc5411.lib.utility.Profile<Keybindings,Preferences>(("DEV_DRIVER"))
        .add(Preferences.CONTROL_EFFORT_X, (Supplier<Double>) () -> -Control.DRIVER_CONTROLLER.getRawAxis((1)))
        .add(Preferences.CONTROL_ZONE_X, (2e-1D))
        .add(Preferences.CONTROL_EFFORT_Y, (Supplier<Double>) () -> -Control.DRIVER_CONTROLLER.getRawAxis((0)))
        .add(Preferences.CONTROL_ZONE_Y, (2e-1D))
        .add(Preferences.CONTROL_EFFORT_T, (Supplier<Double>) () -> -Control.DRIVER_CONTROLLER.getRawAxis((4)))
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
    Profiles(final Profile<Keybindings,Preferences> Profile) {
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