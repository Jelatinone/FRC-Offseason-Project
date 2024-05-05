//--------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.robot2024;
//-------------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.schema.Singleton;
import org.frc5411.lib.schema.thread.CTREOdometryThread;
import org.frc5411.lib.schema.thread.REVOdometryThread;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.estimation.OpenCVHelp;

import org.frc5411.lib.schema.thread.OdometryThread;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
//------------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
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
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  private static volatile Robot Instance;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Robot Constructor.
   */
  private Robot() {} static {
    REVOdometryThread.getInstance();
    CTREOdometryThread.getInstance();
    OpenCVHelp.forceLoadOpenCV();
  }
  //----------------------------------------------------------------------[Robot Scope]------------------------------------------------------------------------//
  @Override
  public synchronized void robotInit() {
    setThreadsEnabled((true));
  }

  @Override
  public synchronized void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  //--------------------------------------------------------------------[Simulation Scope]--------------------------------------------------------------------//
  @Override
  public synchronized void simulationInit() {}

  @Override
  public synchronized void simulationPeriodic() {}

  //---------------------------------------------------------------------[Disabled Scope]--------------------------------------------------------------------//
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
  //--------------------------------------------------------------------[Autonomous Scope]-------------------------------------------------------------------//
  
  @Override
  public synchronized void autonomousInit() {}

  @Override
  public synchronized void autonomousPeriodic() {}

  @Override
  public synchronized void autonomousExit() {}
  //-------------------------------------------------------------------[Teleoperated Scope]------------------------------------------------------------------//
  @Override
  public synchronized void teleopInit() {}

  @Override
  public synchronized void teleopPeriodic() {}

  @Override
  public synchronized void teleopExit() {}

  //-----------------------------------------------------------------------[Test Scope]-----------------------------------------------------------------------//
  @Override
  public synchronized void testPeriodic() {}

  @Override
  public synchronized void testInit() {}

  @Override
  public synchronized void testExit() {}
  //------------------------------------------------------------------------[Methods]------------------------------------------------------------------------//
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
  public final Object clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(("Singleton Instances Cannot Be Cloned"));
  }

  /**
   * Mutates the current state of the running {@link OdometryThread OdometryThreads} to control if they are enabled
   * through {@link OdometryThread#set(Boolean)}.
   * @param Enabled If this Thread is enabled or not
   */
  private synchronized void setThreadsEnabled(final Boolean Enabled) {
    synchronized(Robot.class) {
      REVOdometryThread.getInstance().set(Enabled);
      CTREOdometryThread.getInstance().set(Enabled);
    }
  }
  //---------------------------------------------------------------------[Accessors]-----------------------------------------------------------------------//
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