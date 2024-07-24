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
//-------------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//
import org.frc5411.lib.schema.Singleton;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;

import static org.frc5411.robot2024.Constants.Identity.*;
import static org.frc5411.robot2024.Constants.Mode.*;
//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 *
 *
 * <h1>Robot</h1>
 *
 * <p>Utility class which defines all modes of robot's event-cycle throughout it's lifetime.
 *
 * @see Manager
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public final class Robot extends LoggedRobot implements Singleton<Robot> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  static long serialVersionUID = 9197360083967213848L;

  Map<String,Integer> COMMANDS;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile Robot Instance;

  @NonFinal volatile Command Autonomous;
  @NonFinal volatile Boolean Message;
  @NonFinal volatile Double Timestamp;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Robot Constructor.
   */
  private Robot() {
    COMMANDS = new HashMap<>();
  } static {
    Logger.recordMetadata(("Robot-Type"), TYPE.name());
    Logger.recordMetadata(("Robot-Mode"), MODE.name());
    Logger.recordMetadata(("Runtime-Type"), getRuntimeType().name());
    Logger.recordMetadata(("Robot-Number"), String.valueOf(RobotController.getTeamNumber()));
    Logger.recordMetadata(("Project-Name"), Metadata.MAVEN_NAME);
    Logger.recordMetadata(("Project-Date"), Metadata.BUILD_DATE);
    Logger.recordMetadata(("VCS-SHA"), Metadata.GIT_SHA);
    Logger.recordMetadata(("VCS-Revision"), String.valueOf(Metadata.GIT_REVISION));
    Logger.recordMetadata(("VCS-Date"), Metadata.GIT_DATE);
    Logger.recordMetadata(("VCS-Branch"), Metadata.GIT_BRANCH);
    Logger.recordMetadata(("VCS-State"), switch(Metadata.DIRTY) {
      case (0) -> ("Committed"); case (1) -> ("Changed"); default -> ("Unknown");
    });
  }
  //----------------------------------------------------------------------[Robot Scope]------------------------------------------------------------------------//
  @Override
  public synchronized void robotInit() {
    MANAGEABLE
      .forEach(Supplier::get); 
    Manager.getInstance();       
    CommandScheduler
      .getInstance()
      .onCommandInitialize(
        (Operation) -> log(Operation, (true)));
    CommandScheduler
      .getInstance()
      .onCommandFinish(
        (Operation) -> log(Operation, (false)));
    CommandScheduler
      .getInstance()
      .onCommandInterrupt(
        (Operation) -> log(Operation, (false)));    
    Logger.registerURCL(URCL.startExternal());
    DriverStation.silenceJoystickConnectionWarning((true));
    PortForwarder.add(
      (5800), 
      ("photonvision.local"), 
      (5800));    
    switch(MODE) {
      case ANONYMOUS:
        break;          
      case ACTUAL:
        Logger.addDataReceiver(new WPILOGWriter());
      case SIMULATED:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming((false));
        final var Path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(Path));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(Path, ("-Simulated")), (1e-2)));
        break;
    } if(!MODE.equals(ANONYMOUS)) {
      Shuffleboard.startRecording();
      Logger.start();
      DataLogManager.start();  
    }
  }

  @Override
  public synchronized void robotPeriodic() {
    synchronized(Instance) {
      Threads.setCurrentThreadPriority((true), (99));
      SmartDashboard.updateValues();
      CommandScheduler
        .getInstance()
        .run();
      Manager
        .tryInstance()
        .ifPresent(Manager::update);
      if (Autonomous != (null)) {
        if (!Autonomous.isScheduled() && !Message) {
          System.out.printf(
            ("*** Auto %s in %.2f secs ***%n"),
            DriverStation.isAutonomousEnabled()? "finished": "cancelled",
            Logger.getRealTimestamp() / (1E6D) - Timestamp);
          Message = (true);
        }
      }
      Threads.setCurrentThreadPriority((true), (10));      
    }
  }
  //--------------------------------------------------------------------[Simulation Scope]---------------------------------------------------------------------//
  @Override
  public synchronized void simulationInit() {}

  @Override
  public synchronized void simulationPeriodic() {}
  //---------------------------------------------------------------------[Disabled Scope]----------------------------------------------------------------------//
  @Override
  public synchronized void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public synchronized void disabledPeriodic() {}

  @Override
  public synchronized void disabledExit() {} 
  //--------------------------------------------------------------------[Autonomous Scope]---------------------------------------------------------------------//
  
  @Override
  public synchronized void autonomousInit() {
    Timestamp = Timer.getFPGATimestamp();
    Message = (false);
    if(Autonomous != null) {
      Autonomous
        .onlyWhile(this::isAutonomousEnabled)
        .schedule();
    }
  }

  @Override
  public synchronized void autonomousPeriodic() {}

  @Override
  public synchronized void autonomousExit() {

  }
  //-------------------------------------------------------------------[Teleoperated Scope]--------------------------------------------------------------------//
  @Override
  public synchronized void teleopInit() {}

  @Override
  public synchronized void teleopPeriodic() {}

  @Override
  public synchronized void teleopExit() {}

  //-----------------------------------------------------------------------[Test Scope]------------------------------------------------------------------------//
  @Override
  public synchronized void testPeriodic() {}

  @Override
  public synchronized void testInit() {}

  @Override
  public synchronized void testExit() {}
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Serial
  @Override
  public synchronized Robot readResolve() {
    return Instance;
  }

  @Serial
  @Override
  public synchronized void readObject(final ObjectInputStream Stream) throws IOException, ClassNotFoundException {
    Stream.defaultReadObject();
    Instance = (this);
  }

  @Override
  public synchronized void close() {
    super.close();
    Manager
      .tryInstance()
      .ifPresent(Manager::close);
    synchronized(Robot.class) {
      Logger.end();
      Shuffleboard.stopRecording();
      COMMANDS.clear();
      Instance = (null);
    }
  }

  @Override
  public Robot clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(
      String.format(
        ("%s Instances Cannot Be Cloned"), 
        getClass()
          .getSimpleName()));
  }

  /**
   * Logs a command that has been scheduled with the {@link CommandScheduler} using the {@link Logger}.
   * @param Operation Command to be logged, can be in any state
   * @param Running   Whether this command is currently active
   */
  private void log(final Command Operation, final Boolean Running) {
    final var Name = Operation.getName();
    final var Count = COMMANDS.getOrDefault(Name, (0)) + (Running? 1: -1);
    COMMANDS.put(Name, Count);
    Logger.recordOutput(String.format(("Commands/Unique/[%s]-[%s]"), Name, Integer.toHexString(Operation.hashCode())), Running);
    Logger.recordOutput(String.format(("Commands/Unique/[%s]"), Name), Count > 0);
  }
  //---------------------------------------------------------------------[Mutators]----------------------------------------------------------------------------//
  /**
   * Mutates the current autonomous command to a different command, immediately ends any running commands if applicable.
   * @param Operation Command to be executed, can be in any state, will be run as {@link Command#asProxy() proxy}
   */
  public synchronized void set(final Command Operation) {
    if(Autonomous != (null)) {
      Autonomous.cancel();
    }
    Autonomous = Operation.asProxy();
  }
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Attempts retrieval an instance of this {@link Singleton}, but does not explicitly create a new instance if one does not yet exist
   * @return This singleton's instance, optionally
   */
  public static synchronized Optional<Robot> tryInstance() {
    return Optional
      .ofNullable(Instance);
  }

  /**
   * Retrieves an instance of this {@link Singleton}, or (thread-safely) creates a new instance of this type if an instance has not yet been constructed
   * @return This singleton's instance, guaranteed
   */
  public static synchronized Robot getInstance() {
    Robot Result = Instance;
    if(Instance == (null)) {
      synchronized(Robot.class) {
        Result = Instance;
        if(Instance == (null)) {
          Instance = Result = new Robot();
        }
      }
    }
    return Result;
  }
}