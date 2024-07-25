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
package org.frc5411.robot2024.subsystems.vision;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.instrument.camera.Camera;
import org.frc5411.lib.instrument.camera.archetype.LimelightCamera;
import org.frc5411.lib.instrument.camera.archetype.MockCamera;
import org.frc5411.lib.pattern.Component;
import org.frc5411.lib.schema.Registrable;
import org.frc5411.lib.schema.Singleton;
import org.frc5411.lib.schema.Subsystem;
import org.frc5411.lib.utility.Aggregator;
import org.frc5411.lib.utility.Vector;
import org.frc5411.robot2024.Manager;
import org.frc5411.robot2024.Manager.FieldObservation;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.NamedCommands;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.OpenCVHelp;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.stream.Stream;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;
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
public class VisionSubsystem extends Subsystem {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial 
  static long serialVersionUID = 2571418245449373564L;
  static ReadWriteLock SUBSYSTEM_LOCK;
  static Aggregator<Double> DISCRETE_AGGREGATOR;
  //-----------------------------------------------------------------------[Hardware]--------------------------------------------------------------------------//
  Vector<Camera<?>,N2> CAMERAS;
  Camera<?> IDENTITY;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile VisionSubsystem Instance;
  
  @NonFinal volatile State Mode;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Drivebase Subsystem Constructor.
   */
  private VisionSubsystem() {
    super(SUBSYSTEM_LOCK, ("Vision-Subsystem"));
    CAMERAS = Vector.fill(
      Stream.of(Constants.Cameras.values())
        .map((Camera) ->
          RobotBase.isReal()?
            Camera.get()
              .complete(LimelightCamera::new):
            Camera.get()
              .complete(MockCamera::new))
        .toArray(Camera[]::new)
    );
    IDENTITY = CAMERAS
      .stream()
      .findAny()
      .orElseThrow();
    Mode = State.STALE;
    CAMERAS.forEach((Camera) -> 
      addChild(Camera.getIdentity(), Camera));  
    DISCRETE_AGGREGATOR.reset(DISCRETE_AGGREGATOR.attain());
  } static {
    OpenCVHelp.forceLoadOpenCV();    
    PhotonCamera.setVersionCheckEnabled((false));    
    SUBSYSTEM_LOCK = new ReentrantReadWriteLock((true));
    DISCRETE_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1e6, 
      (Previous, Current) -> Current - Previous);
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Serial
  @Override
  public synchronized VisionSubsystem readResolve() {
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
      synchronized(VisionSubsystem.class) {
        CAMERAS
          .stream()
          .parallel()
          .forEach((Camera) -> {
            try {
              Camera.close();
            } catch (final IOException Ignored) {}
          });
        Instance = (null);
        Mode = (null);
      }
    } finally {
      update();
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
      CAMERAS
        .stream()
        .allMatch(Component::getConnection)
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
      synchronized(VisionSubsystem.class) {
        CAMERAS
          .stream()
          .parallel()
          .forEach((Camera) -> {
            Camera
              .periodic();
            if(Camera.getConnection()) {
              Manager
                .tryInstance()
                .ifPresent((Instance) -> 
                  Instance.sample(new FieldObservation(
                    Stream.of(
                        Camera
                          .getReport()
                          .getObservations())
                      .map(List::of)
                      .toList(), 
                    Camera
                      .getMeasurements(), 
                    Camera
                      .getTimestamps(), 
                    Camera
                      .getDescriptor().Position
                        .getTranslation()
                        .toTranslation2d()
                  ))
                );
            }
          });
        Mode = CAMERAS
          .stream()
          .allMatch((Camera) -> 
            Camera.getReport().getMeasurements().length > (0) && Camera.getConnection())?
          State.ACTIVE:
          State.STALE;
      }
    } finally {
      update();
      SUBSYSTEM_LOCK.writeLock().unlock();
    }
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  /**
   * Provides the current position of child {@link Camera camera} of this drivebase as a {@link Pose3d} object
   * <p> Performs a read-lock blocking operation, which ensures that {@link org.frc5411.lib.pattern.Report reports} are up-to-date before retrieval of {@link Camera#getMeasurement() measurement} values
   * @return Estimated robot position at the current time now
   * @implNote It is preferred to obtain chassis, and module related odometry values via the {@link Manager#getFieldOdometry(Double) Manager}
   */
  public Pose3d[] getCameraMeasurements() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return CAMERAS
        .stream()
        .map((Camera) -> 
          Camera
            .getMeasurement()
            .orElse(new Pose3d(
              new Translation3d(Double.NaN, Double.NaN, Double.NaN), 
              new Rotation3d(Double.NaN, Double.NaN, Double.NaN))
            ))
        .toArray(Pose3d[]::new);
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
  }

  /**
   * Provides the current timestamp of all child {@link Camera camera} of this robot
   * <p> Performs a read-lock blocking operation, which ensures that {@link org.frc5411.lib.pattern.Report reports} are up-to-date before retrieval of {@link Camera#getTimestamp() timestamp} values
   * @return Gyroscope measured timestamp (seconds)
   */
  public double[] getCameraTimestamps() {
    try {
      SUBSYSTEM_LOCK.readLock().lock();
      return CAMERAS
        .stream()
        .mapToDouble((Camera) -> 
          Camera
            .getTimestamp()
            .orElse(Double.NaN))
        .toArray();
    } finally {
      SUBSYSTEM_LOCK.readLock().unlock();
    } 
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
  public static synchronized Optional<VisionSubsystem> tryInstance() {
    return Optional
      .ofNullable(Instance);
  }

  /**
   * Retrieves an instance of this {@link Singleton}, or (thread-safely) creates a new instance of this type if an instance has not yet been constructed
   * @return This singleton's instance, guaranteed
   */
  public static synchronized VisionSubsystem getInstance() {
    VisionSubsystem Result = Instance;
    if(Instance == (null)) {
      synchronized(VisionSubsystem.class) {
        Result = Instance;
        if(Instance == (null)) {
          Instance = Result = new VisionSubsystem();
        }
      }
    }
    return Result;
  }
  //-------------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   * <h1>State</h1>
   * 
   * Represents the named states of operation of vision, which have do not have distinct behavior that differentiate it from other modes of control, but
   * represent different modes of logic which occur underneath when reading Camera values
   */
  enum State {
    //------------------------------------------------------------------------[Values]---------------------------------------------------------------------------//
    /**
     * Represents a stale (or waiting) state of this instance, where {@link VisionSubsystem#periodic()} is not running; therefore the subsystem's 
     * measurements are considered 'stale' or out-of-date.
     */
    STALE,
    /**
     * Represents an active (or running) state of this instance, where {@link VisionSubsystem#periodic()} is running; therefore the subsystem's 
     * measurements are considered to be up-to-date (but still possibly in the process of updating).
     */
    ACTIVE
  }
} 
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
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
    VisionSubsystem
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