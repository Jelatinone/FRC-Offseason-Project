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

import org.frc5411.lib.instrument.module.Limit;
import org.frc5411.lib.schema.Singleton;
import org.frc5411.lib.schema.Subsystem;
import org.frc5411.lib.utility.Figures;

import org.frc5411.robot2024.subsystems.drivebase.DrivebaseSubsystem;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.jcabi.aspects.Async;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.urcl.URCL;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.List;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;

import static edu.wpi.first.math.MathUtil.*;
import static org.frc5411.lib.utility.Geometry.*;
import static org.frc5411.robot2024.Constants.Control.*;
import static org.frc5411.robot2024.Constants.Field.*;
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
@SuppressWarnings("unused")
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public final class Manager implements Singleton<Manager> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  static long serialVersionUID = 2389697764281159320L;
  
  static Vector<N2> STATE_STANDARD_DEVIATIONS = VecBuilder.fill((1D),(1D));
  static Vector<N2> MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill((1D),(1D));

  static Double VISION_CORRECTION = (2D);

  static Integer CHASSIS_CAPACITY;

  static ReadWriteLock UPDATE_LOCK;

  ExecutorService CALLBACK;

  TimeInterpolatableBuffer<Pose2d> VEHICLE_ODOMETRY;
  TimeInterpolatableBuffer<Translation2d> FIELD_ODOMETRY;

  Limit LIMITS;
  SwerveDriveKinematics KINEMATICS;
  SwerveDriveOdometry ODOMETRY;  

  ExtendedKalmanFilter<N2,N2,N2> FILTER;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile Manager Instance;

  static volatile Double Timestamp;
  static volatile Twist2d Measured;
  static volatile Twist2d Predicted;

  static volatile SwerveModulePosition[] Position;
  static volatile Rotation2d Rotation;

  static volatile Optional<VehicleObservation> Vehicle;
  static volatile Optional<VisionObservation> Vision;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Manager Constructor.
   */
  private Manager() {
    CALLBACK = Executors
      .newWorkStealingPool(THREAD_PARALLELISM);
    VEHICLE_ODOMETRY = TimeInterpolatableBuffer
      .createBuffer(BUFFER_SIZE);
    FIELD_ODOMETRY = TimeInterpolatableBuffer
      .createBuffer(BUFFER_SIZE);
    FILTER = new ExtendedKalmanFilter<>(
      Nat.N2(),
      Nat.N2(),
      Nat.N2(),
      (Input, Output) -> Output,
      (Input, Output) -> Input,
      STATE_STANDARD_DEVIATIONS,
      MEASUREMENT_STANDARD_DEVIATIONS,
      1D / UPDATE_FREQUENCY);
    Position = DrivebaseSubsystem
      .tryInstance()
      .map(DrivebaseSubsystem::getModulePositions)
      .orElse(
        IntStream
          .range((0), DrivebaseSubsystem.getCapacity())
          .mapToObj((Value) -> 
            new SwerveModulePosition(Double.NaN, Rotation2d.fromRotations(Double.NaN)))
          .toArray(SwerveModulePosition[]::new));
    Rotation = DrivebaseSubsystem
      .tryInstance()
      .map((Instance) -> 
        Instance.getGyroscopePosition().toRotation2d())
      .orElse(Rotation2d.fromRotations(Double.NaN));
    Vehicle = Optional
      .empty();
    Vision = Optional
      .empty();
    LIMITS = DrivebaseSubsystem.getLimits();
    KINEMATICS = DrivebaseSubsystem.getKinematics();
    ODOMETRY = DrivebaseSubsystem
      .tryInstance()
      .map(DrivebaseSubsystem::getOdometry)
      .orElse(new SwerveDriveOdometry(KINEMATICS, Rotation, Position));
    CHASSIS_CAPACITY = Position.length;  
    configure();
    Robot
      .tryInstance()
      .ifPresent((Instance) -> 
        Instance.add(
          () -> {
            if(Manager.Instance != (null)) {
              Manager.Instance.update();
            }
          },
          UPDATE_FREQUENCY
        )
      );
  } static {
    UPDATE_LOCK = new ReentrantReadWriteLock((true));
    Measured = new Twist2d();
    Predicted = new Twist2d();
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
  public synchronized void close() {
    synchronized(Manager.class) {
      Subsystem.getSubsystems().forEach((Subsystem) -> {
        try {
          Subsystem.close();
        } catch(final IOException Ignored) {}
      });      
      Instance = (null);
    }
  }

  /**
   * Performs all configurations, for all subsystems, on all valid operators of this subsystem.
   */
  private synchronized void configure() { 
    DrivebaseSubsystem
      .tryInstance()
      .ifPresent((Instance) -> {
        Instance
          .setDefaultCommand(
            new InstantCommand(() -> Instance.apply(new Twist2d(
              applyDeadband(
                DRIVER
                  .<Supplier<Double>>getPreference(CONTROL_EFFORT_X)
                  .orElse((() -> 0D))
                  .get(), 
                DRIVER
                  .<Double>getPreference(CONTROL_ZONE_X)
                  .orElse((0D))),
              applyDeadband(
                DRIVER
                  .<Supplier<Double>>getPreference(CONTROL_EFFORT_Y)
                  .orElse((() -> 0D))
                  .get(), 
                DRIVER
                  .<Double>getPreference(CONTROL_ZONE_Y)
                  .orElse((0D))),
              applyDeadband(
                DRIVER
                  .<Supplier<Double>>getPreference(CONTROL_EFFORT_T)
                  .orElse((() -> 0D))
                  .get(), 
                DRIVER
                  .<Double>getPreference(CONTROL_ZONE_T)
                  .orElse((0D)))
                )
              ),
          Instance
        ));
      });
  }

  /**
   * Updates relevant {@link Logger loggable} values using {@link Logger#recordOutput(String, edu.wpi.first.util.WPISerializable)} that may have changed during runtime. This
   * is inclusive of values such as encoder values, motor outputs, etc., that are not automatically logged (such as {@link URCL}) that may be useful during
   * the debugging process.
   */
  @Async
  public synchronized void update() {
    if(Vehicle.isPresent()) {
      Logger.recordOutput(
        ("Robot/Odometry/Vehicle"), 
        getVehicleRelative()
          .getValue()
      );      
    }
    if(Vision.isPresent()) {
      Logger.recordOutput(
        ("Robot/Odometry/Field"), 
        getFieldRelative()
          .getValue()
      );      
    }

    Logger.recordOutput(
      ("Robot/Measured"), 
      Measured
    );
    Logger.recordOutput(
      ("Robot/Predicted"), 
      Predicted
    );
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
   * Adds a new velocity prediction based on the provided {@link ChassisSpeeds speeds}; which must be properly bounded such that no individual desired module speed is
   * greater than it's limits, or that the speeds themselves do not exceed the limits of the drivebase
   * @param Demand Desired speeds of the drivebase's inputs, which have been properly configured
   */
  public synchronized void sample(final ChassisSpeeds Demand) {
    try {
      UPDATE_LOCK.writeLock().lock();
      Predicted = log(
        exp(new Twist2d(
          Demand.vxMetersPerSecond, 
          Demand.vyMetersPerSecond, 
          Demand.omegaRadiansPerSecond))
        .rotateBy(Rotation));      
    } finally {
      UPDATE_LOCK.writeLock().unlock();
    }
  }


  /**
   * <p>Adds a new {@link VehicleObservation observation} instance, containing the necessary information to calculate accurate wheel odometry values, to the callback's work-stealing 
   * execution thread-pool.
   * <p>Note that because of the lightweight nature of this method, with no blocking operations, several repeat calls of this method can be made, with no performance
   * impacts on the robot-main thread.
   * @param Observation Vehicle observation to add to the processing pool
   * @see #sample(VisionObservation)
   * @return Future representing the pending completion of the observation
   */
  public synchronized Future<?> sample(final VehicleObservation Observation) {
    Objects.requireNonNull(Observation);
    return CALLBACK
      .submit(() -> resolve(Observation));
  }  

  /**
   * Updates wheel-base odometry given the provided observation's set of measurements
   * @param Observation Pooled value, which has not yet been processed, and represents a set of measurements that occurred over an interval of time
   */
  @Async
  private synchronized void resolve(final VehicleObservation Observation) {
    try {
      UPDATE_LOCK.writeLock().lock();
      final var Updates = Figures
        .minimum(Observation.Positions().size(), Observation.Timestamps().size());
      for(var Update = (0); Update < Updates; Update++) {
        var Include = (true);
        final var Delta = Observation.Timestamps().get(Update) - Timestamp;
        final var Positions = new SwerveModulePosition[CHASSIS_CAPACITY];
        final var Deltas = new SwerveModulePosition[CHASSIS_CAPACITY];
        for(var Module = (0); Module < CHASSIS_CAPACITY ^ !Include; Module++) {
          Positions[Module] = Observation.Positions().get(Module).get(Update);
          Deltas[Module] = new SwerveModulePosition(
            Positions[Module].distanceMeters - Position[Module].distanceMeters,
            Positions[Module].angle);
          final var Velocity = 
            (Deltas[Module].distanceMeters) / Delta;
          final var Omega = 
            Deltas[Module].angle
              .minus(Positions[Module].angle)
              .div(Delta)
              .getRadians();
          Include =           
            !(Math.abs(Omega) > LIMITS.RotationalVelocity() * (5D) | Math.abs(Velocity) > LIMITS.TranslationalVelocity() * (5D)); 
        }
        if(Include || Vehicle.isEmpty()) {
          Vehicle = Optional
            .of(Observation);          
          Measured = KINEMATICS
            .toTwist2d(Deltas);
          Rotation = !Observation.Rotations().isEmpty() ^ Double.isFinite(Observation.Rotations().get(Update).getRadians())?
            Observation
              .Rotations().get(Update):
            Rotation
              .plus(new Rotation2d(Measured.dtheta));
          VEHICLE_ODOMETRY.addSample(
            Timestamp = Observation
              .Timestamps()
              .get(Update),
            ODOMETRY
              .update(Rotation, Position = Positions)
          );       
          FILTER.predict(
            VecBuilder
              .fill((0D), (0D)),
            Delta
          );             
        }    
      }
    } finally {
      UPDATE_LOCK.writeLock().unlock();
    }
  }

  /**
   * <p>Adds a new {@link VisionObservation observation} instance, containing the necessary information to calculate accurate vision values, to the callback's work-stealing 
   * execution thread-pool.
   * <p>Note that because of the lightweight nature of this method, with no blocking operations, several repeat calls of this method can be made, with no performance
   * impacts on the robot-main thread.
   * @param Observation Vision observation to add to the processing pool
   * @see #sample(VehicleObservation)
   */
  public synchronized Future<?> sample(final VisionObservation Observation) {
    Objects.requireNonNull(Observation);
    return CALLBACK
      .submit(() -> resolve(Observation));
  }

  /**
   * Updates vision odometry given the provided observation's measurement
   * @param Observation Pooled value, which has not yet been processed, and represents a single measurement that occurred at a given point in time
   */
  @Async
  private synchronized void resolve(final VisionObservation Observation) {
    try {
      UPDATE_LOCK.writeLock().lock();
      final var Updates = Figures
        .minimum(Observation.Positions().size(), Observation.Timestamps().size());
      final var Approximate = getVehicleRelative()
        .getValue()
        .getTranslation();
      for(var Update = (0); Update < Updates; Update++) {
        final var Position = Observation
          .Positions()
          .get(Update);
        final Translation2d Camera = Position
          .toPose2d()
          .getTranslation()
          .plus(
            Observation
              .Camera()
              .getTranslation()
              .rotateBy(getVehicleRotation()));
        final var Vehicle = getVehicleRelative()
          .getValue()
          .getTranslation();
        final var Field = Camera
          .plus(Approximate.unaryMinus());
        if(Vision.isEmpty()) {
          FIELD_ODOMETRY.addSample(
            Observation
              .Timestamps()
              .get(Update),
            Field);
          FILTER.setXhat(VecBuilder
            .fill(
              Field.getX(),
              Field.getY()
          ));          
        } else {
          if(
            Math
              .hypot(Measured.dx, Measured.dy) > LIMITS.TranslationalVelocity() 
              &&
            Field.getX() > -MARGIN && Field.getX() < LENGTH + MARGIN && Field.getY() > -MARGIN && Field.getY() < WIDTH + MARGIN
              &&
            Field
              .minus(getFieldRelative().getValue()).getNorm() > VISION_CORRECTION
          ) {
            try {
              final var Distances = Observation
                .Targets()
                .get(Update)
                .stream()
                .map((Target) -> 
                  Target.getTranslation().getNorm())
                .toList();
              final var Minimum = Distances
                .stream()
                .min(Double::compareTo)
                .orElse((0D));
              final var Total = Distances
                .stream()
                .mapToDouble(Double::doubleValue)
                .sum();
              final var Deviation = 
                (1/10D) * (((1/100D) * Math.pow(Minimum, (2D))) + ((1/200D) * Math.pow(Total / Distances.size(), (2D)))) / Distances.size();
              FILTER.correct(
                VecBuilder.fill(
                    (0D), 
                    (0D)),
                VecBuilder.fill(
                    Field.getX(),
                    Field.getY()),
                StateSpaceUtil.makeCovarianceMatrix(
                  Nat.N2(), 
                  VecBuilder.fill(
                    Math.pow((Deviation), (1D)), 
                    Math.pow((Deviation), (1D))
                  )
                )
                  
              );
              FIELD_ODOMETRY.addSample(
                Timestamp = Observation
                  .Timestamps()
                  .get(Update), 
                new Translation2d(
                  new Vector<N2>(FILTER.getXhat())));
            } catch(final Exception Ignored) {}
          }
        }
      }
      Vision = Optional
        .of(Observation);
    } finally {
      UPDATE_LOCK.writeLock().unlock();
    }
  }
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Provides the vehicle odometry at the given time provided, which is an estimate based upon the {@link #sample(VehicleObservation) addition} of 
   * {@link VehicleObservation wheel observations}
   * @param Timestamp Time at which to obtain a sample of vehicle odometry
   * @return Robot (vehicle-relative) odometry position at the given time
   */
  public Entry<Double,Pose2d> getVehicleRelative(final Double Timestamp) {
    try {
      UPDATE_LOCK.readLock().lock();
      return (null); // <--- TODO: Prediction Logic
    } finally {
      UPDATE_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the vehicle odometry at the current time provided, which is an estimate based upon the {@link #sample(VehicleObservation) addition} of
   * {@link VehicleObservation wheel observations}
   * @return Robot (vehicle-relative) odometry position
   */
  public Entry<Double,Pose2d> getVehicleRelative() {
    try {
      UPDATE_LOCK.readLock().lock();
      return VEHICLE_ODOMETRY
        .getInternalBuffer()
        .lastEntry();
    } finally {
      UPDATE_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the field odometry at the given time provided, which is an estimate based upon the {@link #sample(VisionObservation) addition} of 
   * {@link VehicleObservation wheel observations}
   * @param Timestamp Time at which to obtain a sample of field odometry
   * @return Robot (field-relative) odometry position at the given time
   */
  public Entry<Double,Translation2d> getFieldRelative(final Double Timestamp) {
    try {
      UPDATE_LOCK.readLock().lock();
      return (null); // <--- TODO: Prediction Logic
    } finally {
      UPDATE_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the field odometry at the current time provided, which is an estimate based upon the {@link #sample(VisionObservation) addition} of
   * {@link VehicleObservation wheel observations}
   * @return Robot (field-relative) odometry position
   */
  public Entry<Double,Translation2d> getFieldRelative() {
    try {
      UPDATE_LOCK.readLock().lock();
      return FIELD_ODOMETRY
        .getInternalBuffer()
        .lastEntry();
    } finally {
      UPDATE_LOCK.readLock().unlock();
    }
  }

  /**
   * Provide the rotation, along the z-axis (yaw) of the drivebase as a {@link Rotation2d rotation} object.
   * @return Rotation along the z-axis (yaw) observed by the robot
   */
  public Rotation2d getVehicleRotation() {
    try {
      UPDATE_LOCK.readLock().lock();
      return Rotation;
    } finally {
      UPDATE_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the relative position of the chassis' wheel's positions (where the relatively depends upon underlying implementation)
   * @return Position of wheels jn two-dimensional space observed by the robot
   */
  public SwerveModulePosition[] getVehiclePosition() {
    try {
      UPDATE_LOCK.readLock().lock();
      return Position;
    } finally {
      UPDATE_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the measured velocity determined via the {@link #sample(VehicleObservation) addition} of {@link VehicleObservation observations}
   * @return Measured velocity, calculated by the delta between the most recent positions
   */
  public Twist2d getMeasured() {
    try {
      UPDATE_LOCK.readLock().lock();
      return Measured;
    } finally {
      UPDATE_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the predicted velocity determined via the {@link #sample(ChassisSpeeds) addition} of {@link ChassisSpeeds speeds}
   * @return Predicted velocity, calculated by the most recently provided speeds
   */
  public Twist2d getPredicted() {
    try {
      UPDATE_LOCK.readLock().lock();
      return Predicted;
    } finally {
      UPDATE_LOCK.readLock().unlock();
    }
  } 

  /**
   * Provides the latest vehicle observation {@link #sample(VehicleObservation) sampled}.
   * @return Latest vehicle observation
   */
  public Optional<VehicleObservation> getVehicleObservation() {
    return Vehicle;
  }

  /**
   * Provides the latest vision observation {@link #sample(VisionObservation) sampled}.
   * @return Latest vision observation
   */
  public Optional<VisionObservation> getVisionObservation() {
    return Vision;
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
   * <h1>VisionObservation</h1>
   *
   */
  public record VisionObservation(List<List<Transform3d>> Targets, List<Pose3d> Positions, List<Double> Timestamps, Transform2d Camera) {}
}