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
import org.frc5411.lib.schema.thread.CTREOdometryThread;
import org.frc5411.lib.schema.thread.OdometryThread;
import org.frc5411.lib.schema.thread.REVOdometryThread;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;
import org.photonvision.estimation.OpenCVHelp;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.HashMap;
import java.util.Map;
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
public final class Robot extends LoggedRobot implements Singleton<Robot> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  private static final long serialVersionUID = 9197360083967213848L;
  private static final Map<String,Integer> COMMANDS = new HashMap<>();
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  private static volatile Robot Instance;
  private static volatile Command Autonomous;
  private static volatile Boolean Message;
  private static volatile Double Timestamp;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Robot Constructor.
   */
  private Robot() {} static {
    REVOdometryThread.getInstance();
    CTREOdometryThread.getInstance();
    OpenCVHelp.forceLoadOpenCV();
    Logger.recordMetadata(("Robot-Type"), Constants.Robot.TYPE.name());
    Logger.recordMetadata(("Robot-Mode"), Constants.Robot.MODE.name());
    Logger.recordMetadata(("Runtime-Type"), getRuntimeType().name());
    Logger.recordMetadata(("Robot-Number"), String.valueOf(RobotController.getTeamNumber()));
    Logger.recordMetadata(("Project-Name"), Metadata.MAVEN_NAME);
    Logger.recordMetadata(("Project-Date"), Metadata.BUILD_DATE);
    Logger.recordMetadata(("VCS-SHA"), Metadata.GIT_SHA);
    Logger.recordMetadata(("VCS-Revision"), String.valueOf(Metadata.GIT_REVISION));
    Logger.recordMetadata(("VCS-Date"), Metadata.GIT_DATE);
    Logger.recordMetadata(("VCS-Branch"), Metadata.GIT_BRANCH);
    Logger.recordMetadata(("VCS-State"), switch(Metadata.DIRTY) {
      case (0) -> "Committed"; case (1) -> "Changed"; default -> "Unknown";
    });
  }
  //----------------------------------------------------------------------[Robot Scope]------------------------------------------------------------------------//
  @Override
  public synchronized void robotInit() {
    switch(Constants.Robot.MODE) {
      case ANONYMOUS:
        break;
      case ACTUAL:
        Logger.addDataReceiver(new WPILOGWriter());
      case SIMULATED:
        Logger.addDataReceiver(new RLOGServer());
        break;
      case REPLAY:
        setUseTiming((false));
        final var Path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(Path));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(Path, ("-Simulated")), (1e-2)));
        break;
    }
    Logger.start();
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command Operation) -> log(Operation, (true)));
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command Operation) -> log(Operation, (false)));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command Operation) -> log(Operation, (false)));
    DataLogManager.start();
    Logger.registerURCL(URCL.startExternal());
    DriverStation.silenceJoystickConnectionWarning((true));
    PortForwarder.add((5800), ("photoemission.local"), (5800));
    setThreadsEnabled((true));
    Manager.getInstance();
  }

  @Override
  public synchronized void robotPeriodic() {
    synchronized(Instance) {
      Threads.setCurrentThreadPriority((true), (99));
      CommandScheduler.getInstance().run();
      if(isReal()) {
        final var CAN = RobotController.getCANStatus();
        Logger.recordOutput(("CAN/Bus-Off-Count"), CAN.busOffCount);
        Logger.recordOutput(("CAN/Percent-Utilization"), CAN.percentBusUtilization);
        Logger.recordOutput(("CAN/Receive-Error-Count"), CAN.receiveErrorCount);
        Logger.recordOutput(("CAN/Transmit-Error-Count"), CAN.transmitErrorCount);
        Logger.recordOutput(("CAN/TX-Count"), CAN.txFullCount);
      }
      if (Autonomous != (null)) {
        if (!Autonomous.isScheduled() && !Message) {
          System.out.printf(
            ("*** Auto %s in %.2f secs ***%n"),
            DriverStation.isAutonomousEnabled()? "finished": "cancelled",
            Logger.getRealTimestamp() / (1e6) - Timestamp);
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
    setThreadsEnabled((false));
  }

  @Override
  public synchronized void disabledPeriodic() {}

  @Override
  public synchronized void disabledExit() {
    setThreadsEnabled((true));
  } 
  //--------------------------------------------------------------------[Autonomous Scope]---------------------------------------------------------------------//
  
  @Override
  public synchronized void autonomousInit() {
    Timestamp = Timer.getFPGATimestamp();
    Message = (false);
    if(Autonomous != null) {
      Autonomous.onlyWhile(this::isAutonomousEnabled).schedule();
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
    Manager.getInstance().close();
    synchronized(Robot.class) {
      Instance = (null);
    }
  }

  @Override
  public Object clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getCanonicalName()));
  }

  /**
   * Logs a command that has been scheduled with the {@link CommandScheduler} using the {@link Logger}.
   * @param Operation Command to be logged, can be in any state
   * @param Running   Whether this command is currently active
   */
  private static void log(final Command Operation, final Boolean Running) {
    final var Name = Operation.getName();
    final var Count = COMMANDS.getOrDefault(Running, (0)) + (Running? 1: -1);
    COMMANDS.put(Name, Count);
    Logger.recordOutput(String.format(("Commands/Unique/[%s]-[%s]"), Name, Integer.toHexString(Operation.hashCode())), Running);
    Logger.recordOutput(String.format(("Commands/Unique/[%s]"),Name), Count > 0);
  }
  //---------------------------------------------------------------------[Mutators]----------------------------------------------------------------------------//

  /**
   * Mutates the current autonomous command to a different command, immediately ends any running commands if applicable.
   * @param Operation Command to be executed, can be in any state, will be run as {@link Command#asProxy() proxy}
   */
  public synchronized void setAutonomousCommand(final Command Operation) {
    if(Autonomous != null) {
      Autonomous.cancel();
    }
    Autonomous = Operation.asProxy();
  }

  /**
   * Mutates the current state of the running {@link OdometryThread OdometryThreads} to control if they are enabled
   * through {@link OdometryThread#set(Boolean)}.
   * @param Enabled If this Thread is enabled or not
   */
  private static synchronized void setThreadsEnabled(final Boolean Enabled) {
    synchronized(Instance) {
      REVOdometryThread.getInstance().set(Enabled);
      CTREOdometryThread.getInstance().set(Enabled);
    }
  }
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
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