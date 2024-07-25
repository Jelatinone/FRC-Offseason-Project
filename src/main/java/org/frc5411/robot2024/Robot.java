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
import org.frc5411.lib.nouveau.PhoenixRegister;
import org.frc5411.lib.nouveau.Register;
import org.frc5411.lib.nouveau.StandardRegister;
//-------------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//
import org.frc5411.lib.schema.Singleton;
import org.frc5411.lib.schema.Subsystem;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
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
  //------------------------------------------------------------------------[Logging]--------------------------------------------------------------------------//
  Map<String,Integer> COMMAND_SCHEDULE;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile Robot Instance;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Robot Constructor.
   */
  private Robot() {
    COMMAND_SCHEDULE = new HashMap<>();
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
    CHILDREN
      .parallelStream()
      .forEach(Supplier::get); 
    StandardRegister
      .tryInstance()
      .ifPresent(Register::start);
    PhoenixRegister
      .tryInstance()
      .ifPresent(Register::start);
    Manager.getInstance();         
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
    CommandScheduler
      .getInstance()
      .cancelAll();
  }

  @Override
  public synchronized void disabledPeriodic() {}

  @Override
  public synchronized void disabledExit() {} 
  //--------------------------------------------------------------------[Autonomous Scope]---------------------------------------------------------------------//
  
  @Override
  public synchronized void autonomousInit() {}

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
    synchronized(Robot.class) {
      super.close();
      StandardRegister
        .tryInstance()
        .ifPresent(StandardRegister::close);
      PhoenixRegister
        .tryInstance()
        .ifPresent(PhoenixRegister::close);
      Manager
        .tryInstance()
        .ifPresent((Instance) -> {
          try {
            Instance.close();
          } catch (final SecurityException Ignored) {}
        });   
      Subsystem
        .getSubsystems()
        .parallelStream()
        .forEach((Child) -> {
            try {
              Child.close();
            } catch (final IOException Ignored) {}
          });
      Logger.end();
      Shuffleboard.stopRecording();
      COMMAND_SCHEDULE.clear();
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
   * @param Active    Whether this command is currently active
   */
  private void log(final Command Operation, final Boolean Active) {
    final var Name = Operation.getName();
    final var Count = COMMAND_SCHEDULE.getOrDefault(Name, (0)) + (Active? 1: -1);
    COMMAND_SCHEDULE
      .put(Name, Count);
    Logger.recordOutput(
      String.format(
        ("Commands/Unique/[%s]-[%s]"), 
          Name, 
          Integer
            .toHexString(Operation.hashCode())), 
      Active);
    Logger.recordOutput(
      String
        .format(
          ("Commands/Unique/[%s]"), 
          Name), 
      Count > 0);
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