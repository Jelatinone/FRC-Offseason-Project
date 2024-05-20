//--------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.robot2024.subsystems.drivebase;
//-------------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.pattern.Component;
import org.frc5411.lib.pattern.actuator.module.Module;
import org.frc5411.lib.schema.Registrable;
import org.frc5411.lib.schema.Subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.NamedCommands;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
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
@FieldDefaults(level = AccessLevel.PRIVATE)
public class DrivebaseSubsystem extends Subsystem<Named,State> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  final static long serialVersionUID = 2571418245449373564L;
  final static Lock SUBSYSTEM_LOCK;
  final static Component<Rotation2d> GYROSCOPE = (null);
  final static Collection<Module<?>> MODULES = List.of(
    
  );
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile DrivebaseSubsystem Instance;
  static volatile State Mode;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * DrivebaseSubsystem Constructor.
   */
  private DrivebaseSubsystem() {
    super(SUBSYSTEM_LOCK, ("Drivebase-Subsystem"));
    MODULES.forEach((Module) -> 
      addChild(String.format(("Module-[%s]"), Module.getPlacement().name()), Module));  
  } static {
    SUBSYSTEM_LOCK = new ReentrantLock((true));
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
   * Provides the current measured (gyroscope) rotation form it's most recent {@link Component#update(org.frc5411.lib.pattern.Report)} cycle
   * @return Gyroscope's current rotation
   */
  public static Rotation2d getMeasuredRotation() {
    return GYROSCOPE.getMeasurement().get();
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
   * Control based on the detection of objects located on the field, i.e. Object-Oriented; driving with respect
   * to game pieces and field elements.
   */
  OBJECTIVE((Translation, Rotation) ->
    null
  ),

  /**
   * Control based on the direction of the absolute rotation (yaw) of the gyroscope , i.e. Field Oriented; driving
   * with respect to the direction of the driverstation on the field
   */
  ABSOLUTE((Translation, Rotation) -> 
    ChassisSpeeds.fromFieldRelativeSpeeds(
      Translation.getX(), 
      Translation.getY(), 
      Rotation.getRadians(), 
      DrivebaseSubsystem.getMeasuredRotation())
  ),

  /**
   * Control based on the relative direction of the robot, i.e. Robot-Oriented; driving with no frame of reference
   * to guide us
   */
  RELATIVE((Translation, Rotation) -> 
    ChassisSpeeds.fromRobotRelativeSpeeds(
      Translation.getX(), 
      Translation.getY(), 
      Rotation.getRadians(), 
      DrivebaseSubsystem.getMeasuredRotation())
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
 * Represents the named, Pathplanner registrable, commands of this subsystem to run along specific points of an .auto PathPlanner file.
 * These are referred to externally in PathPlanner by their {@link #name() enum name}.
 */
enum Named implements Registrable {

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