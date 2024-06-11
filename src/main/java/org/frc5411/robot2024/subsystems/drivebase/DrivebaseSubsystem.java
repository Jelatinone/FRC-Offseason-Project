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
package org.frc5411.robot2024.subsystems.drivebase;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.pattern.Component;
import org.frc5411.lib.pattern.actuator.module.Module;
import org.frc5411.lib.schema.Registrable;
import org.frc5411.lib.schema.Subsystem;
import org.frc5411.lib.utility.Aggregator;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.NamedCommands;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.BiFunction;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 *
 *
 * <h1>DrivebaseSubsystem</h1>
 *
 * <p>Utility class which controls the modules to achieve individual goal set points with an acceptable target range of accuracy and time
 * efficiency and providing an API for querying new goal states.<p>
 * 
 * @see Subsystem
 * @author Cody Washington
 * 
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class DrivebaseSubsystem extends Subsystem<Named,State> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial 
  static long serialVersionUID = 2571418245449373564L;
  static ReadWriteLock SUBSYSTEM_LOCK;
  static Aggregator<Double> DISCRETE_AGGREGATOR;
  //-----------------------------------------------------------------------[Hardware]--------------------------------------------------------------------------//
  Component<Rotation2d> GYROSCOPE;
  Collection<Module<?,?>> MODULES;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile DrivebaseSubsystem Instance;
  static volatile State Mode;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * DrivebaseSubsystem Constructor.
   */
  private DrivebaseSubsystem() {
    super(SUBSYSTEM_LOCK, ("Drivebase-Subsystem"));
    DISCRETE_AGGREGATOR.reset(HALUtil.getFPGATime() / 1e6);
    MODULES = List.of(
      
    );
    GYROSCOPE = (null);
    MODULES.forEach((Module) -> 
      addChild(String.format(("Module-[%s]"), Module.getPlacement().name()), Module));  
    addChild(("Gyroscope"), GYROSCOPE);
    Mode = State.RELATIVE;
  } static {
    SUBSYSTEM_LOCK = new ReentrantReadWriteLock((true));
    DISCRETE_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1e6, 
      (Previous, Current) -> Current - Previous);
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
  public synchronized void close() throws IOException {
    SUBSYSTEM_LOCK.writeLock().lock();
    synchronized(DrivebaseSubsystem.class) {
      MODULES.forEach((Module) -> {
        try {
          Module.close();
        } catch(final IOException Ignored) {}
      });
      try {
        GYROSCOPE.close();
      } catch (final IOException Ignored) {}
      Instance = (null);
      Mode = (null);
      SUBSYSTEM_LOCK.writeLock().unlock();
    }
  }

  @Override
  public synchronized void update() {
    
  }

  @Override
  public synchronized void periodic() {
    try {
      SUBSYSTEM_LOCK.writeLock().lock();
      synchronized(Instance) {
        MODULES.forEach((Module) -> {
          Module.periodic();
          if(DriverStation.isDisabled()) {
            Module.cease();
          }
        });
        GYROSCOPE.periodic();
      }
      update();
    } finally {
      SUBSYSTEM_LOCK.writeLock().unlock();
    }
  }
  //-----------------------------------------------------------------------[Mutators]--------------------------------------------------------------------------//

  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  @Override
  public List<Named> getCommands() {
    return List.of(Named.values());
  }

  @Override
  public State getState() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return Mode;
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
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
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
/**
 * <h1>State</h1>
 * 
 * Represents the named states of operation of the drivebase, which have distinct behavior that differentiate it from other modes of control, i.e.
 * robot-oriented (Relative) control differs from field-oriented through the use of a gyroscope as the reference of rotation.
 */
enum State implements BiFunction<Translation2d, Rotation2d, ChassisSpeeds> {

  /**
   * Control based on the detection of objects located on the field, i.e. Object-Oriented; driving with respect
   * to game pieces and field elements.
   */
  OBJECTIVE((Translation, Rotation) ->
    (null)
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
      (null))
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
      (null))
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
 * <h1>Named</h1>
 * 
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
      NAMED_COMMAND.addRequirements(Instance);
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