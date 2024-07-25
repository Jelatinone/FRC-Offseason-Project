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
import org.frc5411.lib.external.SwerveSetpointGenerator;
import org.frc5411.lib.instrument.gyroscope.Gyroscope;
import org.frc5411.lib.instrument.gyroscope.archetype.PigeonGyroscope;
import org.frc5411.lib.instrument.module.Module;
import org.frc5411.lib.instrument.module.Setpoint;
import org.frc5411.lib.instrument.module.archetype.MockModule;
import org.frc5411.lib.instrument.module.archetype.SparkModule;
import org.frc5411.lib.pattern.Component;
import org.frc5411.lib.schema.Registrable;
import org.frc5411.lib.schema.Singleton;
import org.frc5411.lib.schema.Subsystem;
import org.frc5411.lib.utility.Aggregator;
import org.frc5411.lib.utility.Vector;

import org.frc5411.robot2024.Manager;
import org.frc5411.robot2024.Manager.VehicleObservation;
import org.frc5411.robot2024.subsystems.drivebase.Constants.Modules;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.NamedCommands;

import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Function;
import java.util.stream.Stream;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;

import static org.frc5411.robot2024.subsystems.drivebase.Constants.Identity.*;
import static org.frc5411.robot2024.subsystems.drivebase.Constants.Regulation.*;
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
@FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
public class DrivebaseSubsystem extends Subsystem {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial 
  static long serialVersionUID = 2571418245449373564L;
  static ReadWriteLock SUBSYSTEM_LOCK;
  static Aggregator<Double> DISCRETE_AGGREGATOR;
  //-----------------------------------------------------------------------[Hardware]--------------------------------------------------------------------------//
  Vector<Module<?,?>,N4> MODULES;
  Module<?,?> IDENTITY; 
  Gyroscope<?> GYROSCOPE;
  //----------------------------------------------------------------------[Regulation]-------------------------------------------------------------------------//
  SwerveDriveOdometry ODOMETRY;  
  SwerveSetpointGenerator GENERATOR;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile DrivebaseSubsystem Instance;
  static volatile State Mode;
  static volatile Setpoint Effort;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Drivebase Subsystem Constructor.
   */
  private DrivebaseSubsystem() {
    super(SUBSYSTEM_LOCK, ("Drivebase-Subsystem"));
    MODULES = Vector.fill(
      Stream.of(Modules.values())
        .map((Module) ->
          RobotBase.isReal()?
            Module.get()
              .complete(SparkModule::new):
            Module.get()
              .complete(MockModule::new))
        .toArray(Module[]::new)
    );
    IDENTITY = MODULES
      .stream()
      .findAny()
      .orElseThrow();
    GYROSCOPE = Constants.GYROSCOPE_DESCRIPTOR
      .complete(PigeonGyroscope::new);      
    MODULES
      .forEach(Module::periodic);
    GYROSCOPE
      .periodic();
    ODOMETRY = new SwerveDriveOdometry(
      KINEMATICS, 
      getGyroscopeMeasurement()
        .toRotation2d(), 
      getModuleMeasurements(),
      PRESET
    );
    GENERATOR = SwerveSetpointGenerator
      .builder()
      .kinematics(KINEMATICS)
      .moduleLocations(LOCATIONS)
      .build();
    Mode = State.ABSOLUTE;
    Effort = new Setpoint(
      new ChassisSpeeds(), 
      getModuleStates());
    MODULES.forEach((Module) -> 
      addChild(Module.getIdentity(), Module));  
    addChild(GYROSCOPE.getIdentity(), GYROSCOPE);
    DISCRETE_AGGREGATOR.reset(DISCRETE_AGGREGATOR.attain());
  } static {
    SUBSYSTEM_LOCK = new ReentrantReadWriteLock((true));
    DISCRETE_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1E6D, 
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
    try {
      SUBSYSTEM_LOCK.writeLock().lock();
      synchronized(DrivebaseSubsystem.class) {
        try {
          GYROSCOPE
            .close();
        } catch (final IOException Ignored) {}
        MODULES
          .stream()
          .parallel()
          .forEach((Module) -> {
            try {
              Module
                .close();
            } catch (final IOException Ignored) {}
          });
        Instance = (null);
        Mode = (null);
      }
    } finally {
      SUBSYSTEM_LOCK.writeLock().unlock();
    }
  }

  @Override
  public synchronized void update() {
    Logger.recordOutput(
      String.format(
        ("%s/Nonbound"), getName()),
      IDENTITY
        .getTimestamp()
        .isPresent()
            &
      IDENTITY
        .getMeasurement()
        .isPresent()
    );     
    Logger.recordOutput(
      String.format(
        ("%s/Latency"), getName()),
      DISCRETE_AGGREGATOR
        .attain() 
            - 
      IDENTITY
        .getTimestamp()
        .orElse(Double.NaN)
    );        
    Logger.recordOutput(
      String.format(
        ("%s/Timestamps"), getName()),
      IDENTITY
        .getTimestamps()
        .size()
    );
    Logger.recordOutput(
      String.format(
        ("%s/Measurements"), getName()),
      IDENTITY
        .getMeasurements()
        .size()
    );
    Logger.recordOutput(
      String.format(
        ("%s/Connection"), getName()),
      MODULES
        .stream()
        .allMatch(Component::getConnection)
    );    
    Logger.recordOutput(
      String.format(
        ("%s/Measurement"), getName()),
      getModuleMeasurements()
    );
    Logger.recordOutput(
      String.format(
        ("%s/State"), getName()),
      getModuleStates()
    );
    Logger.recordOutput(
      String.format(
        ("%s/Input"), getName()),
      getModuleInputs()
    );
    Logger.recordOutput(
      String.format(
        ("%s/Output"), getName()),
      getModuleOutputs()
    );
    Logger.recordOutput(
      String.format(
        ("%s/Mode"), getName()),
      getState()
    );
  }

  @Override
  public synchronized void periodic() {
    try {
      SUBSYSTEM_LOCK.writeLock().lock();
      synchronized(DrivebaseSubsystem.class) {
        DISCRETE_AGGREGATOR
          .aggregate();      
        final var Demand = Mode
          .apply(TELEOPERATED_COORDINATOR.update());
        Demand.omegaRadiansPerSecond += HEADING_COORDINATOR.update();
        Effort = GENERATOR.generateSetpoint(
          LIMITS, 
          Effort, 
          ChassisSpeeds.discretize(
            Demand,
            DISCRETE_AGGREGATOR.getAggregated()), 
          DISCRETE_AGGREGATOR.getAggregated());
        GYROSCOPE
          .periodic();
        MODULES
          .stream()
          .parallel()
          .forEach((Module) -> {
            Module.periodic();
            if(DriverStation.isDisabled() || DriverStation.isEStopped()) {
              Module.cease();
            } else {
              Module
                .set(Effort.States()[Module.getDescriptor().Identity.ordinal()]);
            }
          });
        // Manager
        //   .tryInstance()
        //   .ifPresent((Instance) -> 
        //     Instance
        //       .sample(Effort.Speeds())
        //   );
      }
    } finally {
      SUBSYSTEM_LOCK.writeLock().unlock();
      Manager
        .tryInstance()
        .ifPresent((Instance) -> 
          Instance.sample(new VehicleObservation(
            MODULES
              .stream()
              .map(Module::getMeasurements)
              .toList(), 
            IDENTITY
              .getTimestamps(), 
            GYROSCOPE
              .getMeasurements()
              .stream()
              .map(Rotation3d::toRotation2d)
              .toList()))
          );
      update();
    }
  }

  /**
   * Applies an effort (which must be scaled from [-1,+1]) to the drivebase's modules respective translational and rotational actuators upon
   * completion of the next control loop.
   * @param Effort Scaled effort, which has a translation (dx, dy) and rotation (dtheta)
   */
  public synchronized void apply(final Twist2d Effort) {
    try {
      SUBSYSTEM_LOCK.writeLock().lock();
      TELEOPERATED_COORDINATOR
        .coordinate(Objects.requireNonNull(Effort));
      HEADING_COORDINATOR
        .coordinate(Rotation2d.fromRotations(Effort.dtheta));
    } finally {
      SUBSYSTEM_LOCK.writeLock().unlock();
    }
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  /**
   * Provides the current controller state (reference) of all child {@link Module modules} of this drivebase as a {@link SwerveModuleState} object
   * <p> Performs a read-lock blocking operation, which ensures that {@link org.frc5411.lib.pattern.Report reports} are up-to-date before retrieval of {@link Module#getState() reference} values
   * @return Array (ordered) of controller state (reference) of each module
   * @throws java.util.NoSuchElementException One or more modules could not produce a reference value within the last {@link Module#periodic() periodic} cycle, indicative of a hardware error
   */
  public SwerveModuleState[] getModuleStates() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return MODULES
        .stream()
        .map((Module) -> 
          Module
            .getState()
            .orElseThrow())
        .toArray(SwerveModuleState[]::new);
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
  }

  /**
   * Provides the current controller input (effort) of all child {@link Module modules} of this drivebase as a {@link SwerveModuleState} object
   * <p> Performs a read-lock blocking operation, which ensures that {@link org.frc5411.lib.pattern.Report reports} are up-to-date before retrieval of {@link Module#getState() effort} values
   * @return Array (ordered) of controller input (effort) of each module
   * @throws java.util.NoSuchElementException One or more modules could not produce an effort value within the last {@link Module#periodic() periodic} cycle, indicative of a hardware error
   */
  public SwerveModuleState[] getModuleInputs() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return MODULES
        .stream()
        .map((Module) -> 
          Module
            .getInput()
            .orElseThrow())
        .toArray(SwerveModuleState[]::new);
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
  }

  /**
   * Provides the current controller output (measured) of all child {@link Module modules} of this drivebase as a {@link SwerveModuleState} object
   * <p> Performs a read-lock blocking operation, which ensures that {@link org.frc5411.lib.pattern.Report reports} are up-to-date before retrieval of {@link Module#getState() effort} values
   * @return Array (ordered) of controller output (measured) of each module
   * @throws java.util.NoSuchElementException One or more modules could not produce an effort value within the last {@link Module#periodic() periodic} cycle, indicative of a hardware error
   */
  public SwerveModuleState[] getModuleOutputs() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return MODULES
        .stream()
        .map((Module) -> 
          Module
            .getOutput()
            .orElseThrow())
        .toArray(SwerveModuleState[]::new);
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
  }

  /**
   * Provides the current position of all child {@link Module modules} of this drivebase as a {@link SwerveModulePosition} object
   * <p> Performs a read-lock blocking operation, which ensures that {@link org.frc5411.lib.pattern.Report reports} are up-to-date before retrieval of {@link Module#getMeasurement() measurement} values
   * @return Array (ordered) of positions of each module
   * @implNote It is preferred to obtain chassis, and module related odometry values via the {@link Manager#getVehicleOdometry(Double) Manager}
   */
  public SwerveModulePosition[] getModuleMeasurements() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return MODULES
        .stream()
        .map((Module) -> 
          Module
            .getMeasurement()
            .orElse(new SwerveModulePosition(Double.NaN, Rotation2d.fromRotations(Double.NaN))))
        .toArray(SwerveModulePosition[]::new);
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
  }

  /**
   * Provides the current timestamp of all child {@link Module modules} of this drivebase as a {@link SwerveModulePosition} object
   * <p> Performs a read-lock blocking operation, which ensures that {@link org.frc5411.lib.pattern.Report reports} are up-to-date before retrieval of {@link Module#getTimestamp() measurement} values
   * @return Array (ordered) of timestamp of each module
   */
  public double[] getModuleTimestamps() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return MODULES
        .stream()
        .mapToDouble((Module) -> 
          Module
            .getTimestamp()
            .orElse(Double.NaN))
        .toArray();
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    }  
  }

  /**
   * Provides the current position of child {@link Gyroscope gyroscope} of this drivebase as a {@link Rotation3d} object
   * <p> Performs a read-lock blocking operation, which ensures that {@link org.frc5411.lib.pattern.Report reports} are up-to-date before retrieval of {@link Gyroscope#getMeasurement() measurement} values
   * @return Gyroscope measured position on axes x (roll), y (pitch), and z (yaw)
   * @implNote It is preferred to obtain chassis, and module related odometry values via the {@link Manager#getVehicleOdometry(Double) Manager}
   */
  public Rotation3d getGyroscopeMeasurement() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return GYROSCOPE
        .getMeasurement()
        .orElse(new Rotation3d(
          Double.NaN,
          Double.NaN, 
          Double.NaN));
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
  }

  /**
   * Provides the current timestamp of child {@link Gyroscope gyroscope} of this drivebase
   * <p> Performs a read-lock blocking operation, which ensures that {@link org.frc5411.lib.pattern.Report reports} are up-to-date before retrieval of {@link Camera#getTimestamp() timestamp} values
   * @return Gyroscope measured timestamp (seconds)
   */
  public double getGyroscopeTimestamp() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return GYROSCOPE
        .getTimestamp()
        .orElse(Double.NaN);
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
  }

  /**
   * Provides the {@link SwerveDriveOdometry odometry} object of this drivebase chassis instance, which tracks the position and rotation, {@link Pose2d pose}, of the robot
   * chassis.
   * @return Odometry object of this instance
   */
  public SwerveDriveOdometry getOdometry() {
    return ODOMETRY;
  }

  @Override
  public List<Registrable> getCommands() {
    return List
      .of(Named.values());
  }

  /**
   * Provides the enum of the current state of this subsystem instance.
   * @return State of this instance
   */
  public State getState() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return Mode;
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
  }

  /**
   * Attempts retrieval an instance of this {@link Singleton}, but does not explicitly create a new instance if one does not yet exist
   * @return This singleton's instance, optionally
   */
  public static synchronized Optional<DrivebaseSubsystem> tryInstance() {
    return Optional
      .ofNullable(Instance);
  }

  /**
   * Retrieves an instance of this {@link Singleton}, or (thread-safely) creates a new instance of this type if an instance has not yet been constructed
   * @return This singleton's instance, guaranteed
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
  //-------------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   * <h1>Named</h1>
   * 
   * Represents the named, Pathplanner registrable, commands of this subsystem to run along specific points of an .auto PathPlanner file.
   * These are referred to externally in PathPlanner by their {@link #name() enum name}.
   */
  enum Named implements Registrable {
    //------------------------------------------------------------------------[Values]---------------------------------------------------------------------------//
    EMPTY$PLACEHOLDER(new InstantCommand());
    //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
    private final Command NAMED_COMMAND;
    //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
    /**
     * Named Constructor.
     * @param Command Valid named command to register as a {@link NamedCommands NamedCommand}.
     */
    Named(final Command Command) {
      NAMED_COMMAND = Command;
      CommandScheduler
        .getInstance()
        .requireNotComposedOrScheduled(Command);      
      DrivebaseSubsystem
        .tryInstance()
        .ifPresent((Instance) -> {
          if(!NAMED_COMMAND.getRequirements().contains(Instance)) {
            NAMED_COMMAND
              .addRequirements(Instance);
          }
        });
      register();
    }
    //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
    @Override
    public final Command getCommand() {
      return NAMED_COMMAND;
    }

    @Override
    public final String getName() {
      return name();
    }
  }
} 
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
/**
 * <h1>State</h1>
 * 
 * Represents the named states of operation of the drivebase, which have distinct behavior that differentiate it from other modes of control, i.e.
 * robot-oriented (Relative) control differs from field-oriented through the use of a gyroscope as the reference of rotation.
 */
enum State implements Function<Twist2d, ChassisSpeeds> {
  //------------------------------------------------------------------------[Values]---------------------------------------------------------------------------//
  /**
   * Control based on the detection of objects located on the field, i.e. Object-Oriented; driving with respect
   * to game pieces and field elements.
   */
  OBJECTIVE((Twist) -> {
      throw new UnsupportedOperationException();
  }),

  /**
   * Control based on the direction of the absolute rotation (yaw) of the gyroscope , i.e. Field Oriented; driving
   * with respect to the direction of the driver-station on the field
   */
  ABSOLUTE((Twist) -> 
    ChassisSpeeds.fromFieldRelativeSpeeds(
      Twist.dx, 
      Twist.dy, 
      Twist.dtheta, 
      new Rotation2d())
  ),

  /**
   * Control based on the relative direction of the robot, i.e. Robot-Oriented; driving with no frame of reference
   * to guide us
   */
  RELATIVE((Twist) -> 
    ChassisSpeeds.fromRobotRelativeSpeeds(
      Twist.dx, 
      Twist.dy, 
      Twist.dtheta, 
      new Rotation2d())
  );
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  private final Function<Twist2d, ChassisSpeeds> FUNCTION;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * State Constructor.
   * @param Function Bi-function which consumes both the desired rotation and translation to produce speeds for the demand.
   */
  State(final Function<Twist2d, ChassisSpeeds> Function) {
    FUNCTION = Function;
  }
  //-----------------------------------------------------------------------[Mutators]--------------------------------------------------------------------------//
  /**
   * Applies the function's given arguments of Translation and Rotation to create ChassisSpeeds.
   * @param Twist Demand translation & rotation in two-dimensional space
   * @return Output ChassisSpeeds based on the arguments
   */
  public final ChassisSpeeds apply(final Twist2d Twist) {
    return FUNCTION
      .apply(Twist);
  }
}