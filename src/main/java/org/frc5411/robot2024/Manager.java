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
//---------------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.schema.Singleton;
import org.frc5411.lib.schema.Subsystem;
import org.frc5411.lib.utility.Figures;
import org.frc5411.robot2024.subsystems.drivebase.Constants.*;
import org.frc5411.robot2024.subsystems.vision.VisionSubsystem;
import org.frc5411.robot2024.subsystems.drivebase.DrivebaseSubsystem;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.jcabi.aspects.Async;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.urcl.URCL;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.NoSuchElementException;
import java.util.stream.IntStream;
import java.io.Serial;
import java.util.Objects;
import java.util.concurrent.Future;
import java.util.Map.Entry;
import java.util.List;
import java.util.function.Supplier;
import java.util.Collection;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;

import static edu.wpi.first.math.MathUtil.*;
import static org.frc5411.lib.utility.Geometry.exp;
import static org.frc5411.lib.utility.Geometry.log;
import static org.frc5411.robot2024.Constants.Control.*;
import static org.frc5411.robot2024.Constants.Identity.*;
import static org.frc5411.robot2024.Constants.Preferences.*;
//--------------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 *
 *
 * <h1>Manager</h1>
 *
 * <p>Utility class handling the declaration and usage of subsystems at runtime.
 */
@FieldDefaults(level = AccessLevel.PACKAGE, makeFinal = (true))
public final class Manager implements Singleton<Manager> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  static long serialVersionUID = 2389697764281159320L;
  //----------------------------------------------------------------------[Regulation]-------------------------------------------------------------------------//
  ExtendedKalmanFilter<N2,N2,N2> FILTER;  
  Collection<Integer> INDICES;
  SwerveDriveOdometry ODOMETRY;  
  //-----------------------------------------------------------------------[Execution]-------------------------------------------------------------------------//
  ReadWriteLock RESOLUTION_LOCK;
  ExecutorService RESOLUTION_EXECUTOR;
  //-----------------------------------------------------------------------[Odometry]--------------------------------------------------------------------------//
  TimeInterpolatableBuffer<Pose2d> RESOLVED_ODOMETRY;  
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile Manager Instance;

  @NonFinal volatile double Timestamp;
  @NonFinal volatile Twist2d Measured;
  @NonFinal volatile Twist2d Predicted;

  @NonFinal volatile Rotation2d Rotation;
  @NonFinal volatile SwerveModulePosition[] Positions;

  @NonFinal volatile Optional<VehicleObservation> Vehicle;
  @NonFinal volatile Optional<FieldObservation> Field;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Manager Constructor.
   */
  private Manager() {
    RESOLVED_ODOMETRY = TimeInterpolatableBuffer
      .createBuffer(BUFFER_SIZE);
    INDICES = IntStream
      .range((0), Modules.values().length)
      .boxed()
      .toList();
    FILTER = new ExtendedKalmanFilter<>(
      Nat.N2(),
      Nat.N2(),
      Nat.N2(),
      (Input, Output) -> Output,
      (Input, Output) -> Input,
      STATE_STANDARD_DEVIATIONS,
      MEASUREMENT_STANDARD_DEVIATIONS,
      1D / UPDATE_FREQUENCY);
    RESOLUTION_EXECUTOR = Executors
      .newFixedThreadPool(THREAD_PARALLELISM);
    RESOLUTION_LOCK = new ReentrantReadWriteLock((true));
    ODOMETRY = DrivebaseSubsystem
      .tryInstance()
      .map(DrivebaseSubsystem::getOdometry)
      .orElse(new SwerveDriveOdometry(
        Regulation.KINEMATICS, 
        Rotation = (
          DrivebaseSubsystem
            .tryInstance()
            .map((Instance) -> 
              Instance
                .getGyroscopeMeasurement()
                .toRotation2d())
            .orElse(Rotation2d.fromRotations(Double.NaN))
        ),
        Positions = (
          DrivebaseSubsystem
            .tryInstance()
            .map(DrivebaseSubsystem::getModuleMeasurements)
            .orElse(
              INDICES
                .stream()
                .map((Value) -> 
                  new SwerveModulePosition(Double.NaN, Rotation2d.fromRotations(Double.NaN)))
                .toArray(SwerveModulePosition[]::new))
        )
    ));
    Timestamp = HALUtil.getFPGATime() / 1E6D;
    Measured = new Twist2d(
      Double.NaN, 
      Double.NaN, 
      Double.NaN);
    Predicted = new Twist2d(
      Double.NaN, 
      Double.NaN, 
      Double.NaN);
    Vehicle = Optional.empty();
    Field = Optional.empty();
    compose();
  }
  //-----------------------------------------------------------------------[Methods]---------------------------------------------------------------------------//
  @Serial
  @Override
  public synchronized Manager readResolve() {
    return Instance;
  }

  @Serial
  @Override
  public synchronized void readObject(final ObjectInputStream Stream) throws IOException, ClassNotFoundException {
    Stream.defaultReadObject();
    Instance = (this);
  }

  @Override
  public synchronized void close() throws SecurityException {
    try {
      RESOLUTION_LOCK.writeLock().lock();
      synchronized(Manager.class) {
        RESOLUTION_EXECUTOR.shutdownNow();
        RESOLVED_ODOMETRY
          .getInternalBuffer()
          .clear();
        Instance = (null);
      }      
    } finally {
      RESOLUTION_LOCK.writeLock().unlock();
    }
  }

  @Override
  public Manager clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(
      String.format(
        ("%s Instances Cannot Be Cloned"), 
        getClass()
          .getSimpleName()));
  }

  /**
   * 'Composes' (configures) all underlying children {@link Singleton singletons} and {@link Subsystem subsystems} for {@link Robot robot} operation with their respective operator
   * profiles and performs set-up operations against them.
   */
  public synchronized void compose() {
    VisionSubsystem
      .tryInstance()
      .ifPresent(Subsystem::removeDefaultCommand);
    DrivebaseSubsystem
      .tryInstance()
      .ifPresent((Instance) -> Instance
        .setDefaultCommand(
          new InstantCommand(() ->
            Instance.apply(new Twist2d(
              applyDeadband(
                DRIVER
                  .<Supplier<Double>>getPreference(CONTROL_EFFORT_X)
                  .orElse((() -> Double.NaN))
                  .get(),
                DRIVER
                  .<Double>getPreference(CONTROL_ZONE_X)
                  .orElse((Double.NaN))),
              applyDeadband(
                DRIVER
                  .<Supplier<Double>>getPreference(CONTROL_EFFORT_Y)
                  .orElse((() -> Double.NaN))
                  .get(),
                DRIVER
                  .<Double>getPreference(CONTROL_ZONE_Y)
                  .orElse((Double.NaN))),
              applyDeadband(
                DRIVER
                  .<Supplier<Double>>getPreference(CONTROL_EFFORT_T)
                  .orElse((() -> Double.NaN))
                  .get(),
                DRIVER
                  .<Double>getPreference(CONTROL_ZONE_T)
                  .orElse((Double.NaN)))
              )),
            Instance
      )));
  }


  /**
   * Updates relevant {@link Logger loggable} values using {@link Logger#recordOutput(String, edu.wpi.first.util.WPISerializable)} that may have changed during runtime. This
   * is inclusive of values such as encoder values, motor outputs, etc., that are not automatically logged (such as {@link URCL}) that may be useful during
   * the debugging process.
   */
  @Async
  public synchronized void update() {
    if(Vehicle.isPresent() || Field.isPresent()) {
      Logger.recordOutput(
        ("Robot/Odometry"), 
        getResolved()
          .getValue()
      );    
    }     
    Logger.recordOutput(
      ("Robot/Measured"), 
      getMeasured()
    );
    Logger.recordOutput(
      ("Robot/Predicted"), 
      getPredicted()
    );
  }

  /**
   * Adds a new velocity prediction based on the provided {@link ChassisSpeeds speeds}; which must be properly bounded such that no individual desired module speed is
   * greater than it's limits, or that the speeds themselves do not exceed the limits of the drivebase
   * @param Demand Desired speeds of the drivebase's control inputs
   */
  public synchronized void sample(final ChassisSpeeds Demand) {
    try {
      RESOLUTION_LOCK.writeLock().lock();
      Predicted = log(
        exp(new Twist2d(
          Demand.vxMetersPerSecond, 
          Demand.vyMetersPerSecond, 
          Demand.omegaRadiansPerSecond))
        .rotateBy(Rotation));      
    } finally {
      RESOLUTION_LOCK.writeLock().unlock();
    }
  }

  /**
   * <p>Adds a new {@link VehicleObservation observation} instance, containing the necessary information to calculate accurate wheel odometry values, to the callback's work-stealing 
   * execution thread-pool.
   * <p>Note that because of the lightweight nature of this method, with no blocking operations, several repeat calls of this method can be made, with no performance
   * impacts on the robot-main thread.
   * @param Observation Vehicle observation to add to the processing pool
   * @see #sample(FieldObservation)
   * @return Future representing the pending completion of the observation
   */
  public synchronized Future<?> sample(final VehicleObservation Observation) {
    Objects.requireNonNull(Observation);
    return RESOLUTION_EXECUTOR
      .submit(() -> resolve(Observation));
  }  

  /**
   * Resolves the observation made by updating the vehicle-relative odometry values from the values stored by the observation. After which
   * the observation should not be used again.
   * @param Observation Vehicle-relative odometry record (container), which has not yet been {@link #sample(VehicleObservation) sampled}
   */
  @Async
  public synchronized void resolve(final VehicleObservation Observation) {
    try {
      RESOLUTION_LOCK.writeLock().lock();
      IntStream
        .range((0), Figures.minimum(Observation.Positions().size(), Observation.Timestamps().size()))
        .filter((Update) -> {
          try {
            return !(RESOLVED_ODOMETRY.getInternalBuffer().lastKey() - BUFFER_SIZE > Observation.Timestamps().get(Update));
          } catch(final NoSuchElementException Ignored) {
            return Vehicle.isEmpty();
          }
        })
        .forEach((Update) -> {
          final var Interval = Observation.Timestamps().get(Update) - Timestamp;
          final var Position = new SwerveModulePosition[INDICES.size()];
          final var Discrete = new SwerveModulePosition[INDICES.size()];
          final var Included = INDICES
            .stream()
            .allMatch((Module) -> {
              Position[Module] = Observation.Positions().get(Module).get(Update);
              Discrete[Module] = new SwerveModulePosition(
                Position[Module].distanceMeters - Positions[Module].distanceMeters,
                Position[Module].angle);
              final var Velocity = 
                (Discrete[Module].distanceMeters) / Interval;
              final var Omega = 
                Discrete[Module].angle
                  .minus(Position[Module].angle)
                  .div(Interval)
                  .getRadians();
              return 
                !(Math.abs(Omega) > Regulation.LIMITS.RotationalVelocity() * (5D) || Math.abs(Velocity) > Regulation.LIMITS.TranslationalVelocity() * (5D));     
            });
          if(Included || Vehicle.isEmpty()) {
            Measured = Regulation.KINEMATICS.toTwist2d(Discrete);
            Rotation = !Observation.Rotations().isEmpty() ^ Double.isFinite(Observation.Rotations().get(Update).getRadians())?
              Observation
                .Rotations().get(Update):
              Rotation
                .plus(new Rotation2d(Measured.dtheta));
            RESOLVED_ODOMETRY.addSample(
              Timestamp = Observation
                .Timestamps().get(Update),
              ODOMETRY
                .update(Rotation, Positions = Position)
            );      
            FILTER.predict(
              VecBuilder
                .fill((0D), (0D)),
              Interval
            );   
            Vehicle = Optional
              .of(Observation);   
          }
        });
    } finally {
      RESOLUTION_LOCK.writeLock().unlock();
    }
  }

  /**
   * <p>Adds a new {@link FieldObservation observation} instance, containing the necessary information to calculate accurate vision values, to the callback's work-stealing 
   * execution thread-pool.
   * <p>Note that because of the lightweight nature of this method, with no blocking operations, several repeat calls of this method can be made, with no performance
   * impacts on the robot-main thread.
   * @param Observation Vision observation to add to the processing pool
   * @see #sample(VehicleObservation)
   */
  public synchronized Future<?> sample(final FieldObservation Observation) {
    Objects.requireNonNull(Observation);
    return RESOLUTION_EXECUTOR
      .submit(() -> resolve(Observation));
  }

  /**
   * Resolves the observation made by updating the field-relative odometry values from the values stored by the observation. After which
   * the observation should not be used again.
   * @param Observation Field-relative odometry record (container), which has not yet been {@link #sample(FieldObservation) sampled}
   */
  @Async
  public synchronized void resolve(final FieldObservation Observation) {

  }
  //----------------------------------------------------------------------[Accessors]--------------------------------------------------------------------------//
  /**
   * Provides the vehicle odometry at the current time provided, which is an estimate based upon the {@link #sample(VehicleObservation) addition} of
   * {@link VehicleObservation wheel observations}
   * @return Robot (vehicle-relative) odometry position
   */
  public Entry<Double,Pose2d> getResolved() {
    try {
      RESOLUTION_LOCK.readLock().lock();
      return RESOLVED_ODOMETRY
        .getInternalBuffer()
        .lastEntry();      
    } finally {
      RESOLUTION_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the vehicle odometry at the given time provided, which is an estimate based upon the {@link #sample(VehicleObservation) addition} of 
   * {@link VehicleObservation wheel observations}
   * @param Timestamp Time at which to obtain a sample of vehicle odometry
   * @return Robot (vehicle-relative) odometry position at the given time
   * @throws NoSuchElementException When the provided timestamp is out of bounds for the vehicle-relative buffer
   */
  public Pose2d getResolved(final Double Timestamp) {
    try {
      RESOLUTION_LOCK.readLock().lock();
      final var Sample = RESOLVED_ODOMETRY
        .getSample(Timestamp);
      if(Sample.isPresent()) {
        return Sample.get();
      } else {
        if(Timestamp > RESOLVED_ODOMETRY.getInternalBuffer().firstEntry().getKey()) {
          final var Latest = RESOLVED_ODOMETRY
            .getInternalBuffer()
            .lastEntry();
          final var Interval = Timestamp - Latest.getKey();
          return Latest
            .getValue()
            .exp(
              new Twist2d(
                Predicted.dx * Interval, 
                Predicted.dy * Interval, 
                Predicted.dtheta * Interval)
            );  
        } else {
          throw new NoSuchElementException();
        }
      }      
    } finally {
      RESOLUTION_LOCK.readLock().unlock();
    }
  }  

  /**
   * Provides the measured velocity determined via the {@link #sample(VehicleObservation) addition} of {@link VehicleObservation observations}
   * @return Measured velocity, calculated by the delta between the most recent positions
   */
  public Twist2d getMeasured() {
    try {
      RESOLUTION_LOCK.readLock().lock();
      return Measured;
    } finally {
      RESOLUTION_LOCK.readLock().unlock();
    } 
  }

  /**
   * Provides the predicted velocity determined via the {@link #sample(ChassisSpeeds) addition} of {@link ChassisSpeeds speeds}
   * @return Predicted velocity, calculated by the most recently provided speeds
   */
  public Twist2d getPredicted() {
    try {
      RESOLUTION_LOCK.readLock().lock();
      return Predicted;
    } finally {
      RESOLUTION_LOCK.readLock().unlock();
    }
  } 

  /**
   * Provides the latest vehicle observation {@link #sample(VehicleObservation) sampled}.
   * @return Latest vehicle observation
   */
  public Optional<VehicleObservation> getVehicleObservation() {
    try {
      RESOLUTION_LOCK.readLock().lock();
      return Vehicle;
    } finally {
      RESOLUTION_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the latest vision observation {@link #sample(FieldObservation) sampled}.
   * @return Latest vision observation
   */
  public Optional<FieldObservation> getFieldObservation() {
    try {
      RESOLUTION_LOCK.readLock().lock();
      return Field;
    } finally {
      RESOLUTION_LOCK.readLock().unlock();
    }
  }

  /**
   * Attempts retrieval an instance of this {@link Singleton}, but does not explicitly create a new instance if one does not yet exist
   * @return This singleton's instance, optionally
   */
  public static synchronized Optional<Manager> tryInstance() {
    return Optional
      .ofNullable(Instance);
  }

  /**
   * Retrieves an instance of this {@link Singleton}, or (thread-safely) creates a new instance of this type if an instance has not yet been constructed.
   * @return This singleton's instance, guaranteed
   */
  public static synchronized Manager getInstance() {
    Manager Result = Instance;
    if(Instance == (null)) {
      synchronized(Manager.class) {
        Result = Instance;
        if(Instance == (null)) {
          Instance = Result = new Manager();
        }
      }
    }
    return Result;
  }  
  //-----------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   *
   *
   * <h1>VehicleObservation</h1>
   *
   */
  public record VehicleObservation(List<List<SwerveModulePosition>> Positions, List<Double> Timestamps, List<Rotation2d> Rotations) {}

  /**
   *
   *
   * <h1>FieldObservation</h1>
   *
   */
  public record FieldObservation(List<List<Transform3d>> Targets, List<Pose3d> Positions, List<Double> Timestamps, Translation2d Relative) {}
}