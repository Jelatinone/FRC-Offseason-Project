//--------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.robot2024.subsystems.drivebase;
//-------------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.mechanism.actuator.module.Module;
import org.frc5411.lib.schema.Registerable;
import org.frc5411.lib.schema.Subsystem;
import org.frc5411.lib.utility.Operator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.NamedCommands;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
//------------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 *
 *
 * <h1>DrivebaseSubsystem</h1>
 *
 * <p>Utility class which controls the modules to achieve individual goal set points with an acceptable target range of accuracy and time
 * efficiency and providing an API for querying new goal states.<p>
 * 
 * 
 */
public class DrivebaseSubsystem extends Subsystem<Named, State> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  private static final long serialVersionUID = 2571418245449373564L;
  private static final Lock SUBSYSTEM_LOCK;
  private static final Operator<Double> DISCRETE_CLOCK;
  private static final List<Module<?,?>> MODULES;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  private static volatile DrivebaseSubsystem Instance;
  private static volatile State Mode;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * DrivebaseSubsystem Constructor.
   */
  private DrivebaseSubsystem() {
    super(SUBSYSTEM_LOCK, ("Drivebase-Subsystem"));
  } static {
    SUBSYSTEM_LOCK = new ReentrantLock((true));
    DISCRETE_CLOCK = new Operator<>(
      Timer::getFPGATimestamp, 
      (Retained, Source) -> Retained - Source, 
      Timer.getFPGATimestamp());
    MODULES = List.of(

    );
    Mode = State.RELATIVE;
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Serial
  @Override
  public synchronized DrivebaseSubsystem readResolve() {
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
    synchronized(DrivebaseSubsystem.class) {
      Instance = (null);
    }
  }

  @Override
  public synchronized void periodic() {
    SUBSYSTEM_LOCK.lock();
    MODULES.forEach((Module) -> {
      Module.periodic();
      if(DriverStation.isEStopped()) {
        Module.cease();
      }
    });
    DISCRETE_CLOCK.get();

    SUBSYSTEM_LOCK.unlock();
  }

  @Override
  public synchronized void update() {

  }
  //-----------------------------------------------------------------------[Mutators]--------------------------------------------------------------------------//
  
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  @Override
  public List<Named> getCommands() {
    return List.of(Named.values());
  }

  @Override
  public State getState() {
    return Mode;
  }

  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized DrivebaseSubsystem getInstance() {
    DrivebaseSubsystem Result = Instance;
    if(Instance == (null)) {
      synchronized(DrivebaseSubsystem.class) {
        Result = Instance;
        if(Instance == (null)) {
          Instance = Result = new DrivebaseSubsystem();
        }
      }
    }
    return Result;
  }
} 
//-----------------------------------------------------------------------[External]--------------------------------------------------------------------------//
/**
 * Represents the named states of operation of the drivebase, which have distinct behavior that differentiate it from 
 * robot-oriented (Relative) control.
 */
enum State implements BiFunction<Translation2d, Rotation2d, ChassisSpeeds> {

  /**
   * Control based on the detection of objects located on the field, i. e. Object Oriented; driving with respect
   * to game pieces and field elements.
   */
  OBJECTIVE((Translation, Rotation) -> {
    
    return null;
  }),

  /**
   * Control based on the direction of the absolute rotation (yaw) of the gyroscope , i.e. Field Oriented; driving
   * with respect to the direction of the driverstation on the field
   */
  ABSOLUTE((Translation, Rotation) -> 
    ChassisSpeeds.fromFieldRelativeSpeeds(
      Translation.getX(), 
      Translation.getY(), 
      Rotation.getRadians(), 
      null)
  ),

  /**
   * Control based on the relative direction of the robot, i. e. Robot-Oriented; driving with no frame of reference
   * to guide us
   */
  RELATIVE((Translation, Rotation) -> 
    ChassisSpeeds.fromRobotRelativeSpeeds(
      Translation.getX(), 
      Translation.getY(), 
      Rotation.getRadians(), 
      null)
  );

  private final BiFunction<Translation2d, Rotation2d, ChassisSpeeds> FUNCTION;

  /**
   * State Constructor.
   * @param Function Bi-function which consumes both the desired rotation and translation to produce speeds for the demand.
   */
  State(final BiFunction<Translation2d, Rotation2d, ChassisSpeeds> Function) {
    FUNCTION = Function;
  }

  /**
   * Applies the function's given arguments of Translation and Rotation to create ChassisSpeeds.
   * @param Translation Demand translation in two-dimensional space
   * @param Rotation    Demand rotation in two-dimensional space
   * @return Output ChassisSpeeds based on the arguments
   */
  public final ChassisSpeeds apply(final Translation2d Translation, final Rotation2d Rotation) {
    return FUNCTION.apply(Translation, Rotation);
  }
}

/**
 * Represents the named, Pathplanner registerable, commands of this subsystem to run along specific points of an .auto PathPlanner file.
 * These are referred to externally in PathPlanner by their {@link #name() enum name}.
 */
enum Named implements Registerable {

  /**
   * Does nothing, simply a placeholder until real commands are added
   */
  EMPTY_PLACEHOLDER(new InstantCommand());

  private final Command NAMED_COMMAND;

  /**
   * Named Constructor.
   * @param Command Valid named command to register as a {@link NamedCommands NamedCommand}.
   */
  Named(final Command Command) {
    NAMED_COMMAND = Command;
    final var Instance = DrivebaseSubsystem.getInstance();
    if(!NAMED_COMMAND.getRequirements().contains(Instance)) {
      NAMED_COMMAND.addRequirements(DrivebaseSubsystem.getInstance());
    }
    register();
  }

  /**
   * Named Constructor.
   * @param Command Valid runnable operation to register as a {@link NamedCommands NamedCommand}.
   */
  Named(final Runnable Command) {
    NAMED_COMMAND = new InstantCommand(Command, DrivebaseSubsystem.getInstance());
    register();
  }

  @Override
  public final Command getCommand() {
    return NAMED_COMMAND;
  }

  @Override
  public final String getName() {
    return name();
  }
}