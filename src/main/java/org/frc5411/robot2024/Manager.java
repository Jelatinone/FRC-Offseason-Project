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
import org.frc5411.lib.utility.Aggregator;
import org.frc5411.lib.utility.Figures;

import org.frc5411.robot2024.subsystems.drivebase.DrivebaseSubsystem;
import org.frc5411.robot2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.jcabi.aspects.Async;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.urcl.URCL;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.ArrayDeque;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;

import static edu.wpi.first.math.MathUtil.*;
import static org.frc5411.lib.utility.Geometry.*;
import static org.frc5411.robot2024.Constants.Preferences.*;
import static org.frc5411.robot2024.Constants.Identity.*;
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
  static Integer PARALLEL_THREADS = (8);
  static Integer QUEUE_SIZE = (20);
  static Double BUFFER_SIZE = (2D);
  static Double UPDATE_FREQUENCY = (100D);
  static Vector<N2> STATE_STANDARD_DEVIATIONS = VecBuilder.fill((1D),(1D));
  static Vector<N2> MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill((1D),(1D));
  static Aggregator<Double> WHEEL_TIME_AGGREGATOR;
  static Aggregator<Double> VISION_TIME_AGGREGATOR;

  static @NonFinal Integer MODULES;

  static ReadWriteLock WHEEL_UPDATE_LOCK;
  static ReadWriteLock VISION_UPDATE_LOCK;  

  ExecutorService CALLBACK;

  Queue<WheelObservation> WHEEL_UPDATE_QUEUE;
  Queue<VisionObservation> VISION_UPDATE_QUEUE;

  TimeInterpolatableBuffer<Pose2d> VEHICLE_ODOMETRY;
  TimeInterpolatableBuffer<Translation2d> FIELD_ODOMETRY;

  SwerveDriveKinematics KINEMATICS;
  SwerveDriveOdometry ODOMETRY;  
  ExtendedKalmanFilter<N2,N2,N2> FILTER;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile Manager Instance;
  static volatile Twist2d Measured;
  static volatile Twist2d Predicted;
  static volatile Rotation2d Rotation;
  static volatile SwerveModulePosition[] Position;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Manager Constructor.
   */
  private Manager() {
    //<--- Fetch All Managed Subsystems --->
    VisionSubsystem
      .getInstance();      
    DrivebaseSubsystem
      .getInstance();      
    //<--- Initialize Constants --->
    CALLBACK = Executors
      .newWorkStealingPool(PARALLEL_THREADS);
    WHEEL_UPDATE_QUEUE = new ArrayDeque<>(QUEUE_SIZE);
    VISION_UPDATE_QUEUE = new ArrayDeque<>(QUEUE_SIZE);
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

    //<--- Sample from Subsystems --->
    KINEMATICS = DrivebaseSubsystem
      .getKinematics();
    ODOMETRY = DrivebaseSubsystem
      .getInstance()
      .getOdometry();
    Position = DrivebaseSubsystem
      .getInstance()
      .getModulePositions();
    Rotation = DrivebaseSubsystem
      .getInstance()
      .getGyroscopePosition()
      .toRotation2d();

    //<--- Base Sampling --->
    VEHICLE_ODOMETRY.addSample(
      WHEEL_TIME_AGGREGATOR.attain(), 
      ODOMETRY.update(
        DrivebaseSubsystem
          .getInstance()
          .getGyroscopePosition()
          .toRotation2d(), 
        DrivebaseSubsystem
          .getInstance()
          .getModulePositions()
      ));

    //<--- Apply Configurations --->
    configure();
    Robot
      .getInstance()
      .add(
        () -> {
          if(Instance != (null)) {
            Instance.update();
          }
        },
        UPDATE_FREQUENCY);
    MODULES = Position.length;    
  } static {
    //<--- Construct static fields --->
    WHEEL_UPDATE_LOCK = new ReentrantReadWriteLock((true));
    VISION_UPDATE_LOCK = new ReentrantReadWriteLock((true));    
    WHEEL_TIME_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1e6, 
      (Previous, Current) -> Current - Previous);
    VISION_TIME_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1e6, 
      (Previous, Current) -> Current - Previous);
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
      .getInstance()
      .setDefaultCommand(new InstantCommand(() -> 
        DrivebaseSubsystem
          .getInstance()
          .apply(new Twist2d(
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
        DrivebaseSubsystem.getInstance()
    ));
  }

  /**
   * Updates relevant {@link Logger loggable} values using {@link Logger#recordOutput(String, edu.wpi.first.util.WPISerializable)} that may have changed during runtime. This
   * is inclusive of values such as encoder values, motor outputs, etc., that are not automatically logged (such as {@link URCL}) that may be useful during
   * the debugging process.
   */
  @Async
  public synchronized void update() {
    Logger.recordOutput(
      ("Robot/Wheel"),
      WHEEL_UPDATE_QUEUE.size()
    );    
    Logger.recordOutput(
      ("Robot/Vision"),
      VISION_UPDATE_QUEUE.size()
    );
    Logger.recordOutput(
      ("Robot/Vehicle"), 
      getVehicleOdometry()
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
    Predicted = log(
      exp(new Twist2d(
        Demand.vxMetersPerSecond, 
        Demand.vyMetersPerSecond, 
        Demand.omegaRadiansPerSecond))
      .rotateBy(
        getVehicleOdometry().getRotation()));
  }


  /**
   * <p>Adds a new {@link WheelObservation observation} instance, containing the necessary information to calculate accurate wheel odometry values, to the callback's work-stealing 
   * execution thread-pool.
   * <p>Note that because of the lightweight nature of this method, with no blocking operations, several repeat calls of this method can be made, with no performance
   * impacts on the robot-main thread.
   * @param Observation Wheel observation to add to the processing pool
   * @see #sample(VisionObservation)
   * @return Future representing the pending completion of the observation
   */
  public synchronized Future<?> sample(final WheelObservation Observation) {
    return CALLBACK
      .submit(() -> resolve(Objects.requireNonNull(Observation)));
  }  

  /**
   * Updates wheel-base odometry given the provided observation's set of measurements
   * @param Observation Pooled value, which has not yet been processed, and represents a set of measurements that occurred over an interval of time
   */
  @Async
  private synchronized void resolve(final WheelObservation Observation) {
    try {
      WHEEL_UPDATE_LOCK.writeLock().lock();
      WHEEL_TIME_AGGREGATOR.aggregate();
      final var Updates = Figures
        .minimum(Observation.Positions().size(), Observation.Timestamps().size());
      for(int Update = (0); Update < Updates; Update++) {
        FILTER.predict( 
          VecBuilder.fill(
            (0D), 
            (0D)), 
          WHEEL_TIME_AGGREGATOR.getAggregated());     
        final var Positions = new SwerveModulePosition[MODULES];
        final var Deltas = new SwerveModulePosition[MODULES];
        for(int Module = (0); Module < MODULES; Module++) {
          Positions[Module] = Observation.Positions().get(Module).get(Update);
          Deltas[Module] = new SwerveModulePosition(
            Positions[Module].distanceMeters - Position[Module].distanceMeters,
            Positions[Module].angle);
          Position[Module] = Positions[Module];
        }   
        Measured = KINEMATICS.toTwist2d(Deltas);
        Rotation = !Observation.Rotations().isEmpty() ^ Double.isFinite(Observation.Rotations().get(Update).getRadians())?
          Observation
            .Rotations().get(Update):
          Rotation
            .plus(new Rotation2d(Measured.dtheta));
        VEHICLE_ODOMETRY.addSample(
          Observation.Timestamps().get(Update), 
          ODOMETRY.update(Rotation, Positions)
        );                
      }
    } finally {
      WHEEL_UPDATE_LOCK.writeLock().unlock();
    }
  }

  /**
   * <p>Adds a new {@link VisionObservation observation} instance, containing the necessary information to calculate accurate vision values, to the callback's work-stealing 
   * execution thread-pool.
   * <p>Note that because of the lightweight nature of this method, with no blocking operations, several repeat calls of this method can be made, with no performance
   * impacts on the robot-main thread.
   * @param Observation Vision observation to add to the processing pool
   * @see #sample(WheelObservation)
   */
  public synchronized Future<?> sample(final VisionObservation Observation) {
    return CALLBACK
      .submit(() -> resolve(Objects.requireNonNull(Observation)));
  }

  /**
   * Updates vision odometry given the provided observation's measurement
   * @param Observation Pooled value, which has not yet been processed, and represents a single measurement that occurred at a given point in time
   */
  @Async
  private synchronized void resolve(final VisionObservation Observation) {
    try {
      VISION_UPDATE_LOCK.writeLock().lock();
      VISION_TIME_AGGREGATOR.aggregate();
      // <--- TODO: Vision Resolution
    } finally {
      VISION_UPDATE_LOCK.writeLock().unlock();
    }
  }
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Retrieves an instance of this {@link Singleton}, or (thread-safely) creates a new instance of this type if an instance has not yet been constructed.
   * @return This singleton's instance
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

  /**
   * Provides the vehicle odometry at the given time provided, which is an estimate based upon the {@link #sample(WheelObservation) addition} of 
   * {@link WheelObservation wheel observations}
   * @param Timestamp Time at which to obtain a sample of vehicle odometry
   * @return Robot (vehicle)'s odometry position at the given time
   * @throws java.util.NoSuchElementException When a sample does not exist for the provided time
   */
  public Pose2d getVehicleOdometry(final Double Timestamp) {
    try {
      WHEEL_UPDATE_LOCK.readLock().lock();
      return VEHICLE_ODOMETRY
        .getSample(Timestamp)
        .orElseThrow();
      // <--- TODO: Look-ahead Sampling
    } finally {
      WHEEL_UPDATE_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the vehicle odometry at the current time provided by the discretization clock's {@link Aggregator#attain()}, which is an estimate
   * based upon the {@link #sample(WheelObservation) addition} of {@link WheelObservation wheel observations}
   * @return Robot (vehicle)'s odometry position
   * @throws java.util.NoSuchElementException When a sample cannot be found for the current time
   */
  public Pose2d getVehicleOdometry() {
    return getVehicleOdometry(WHEEL_TIME_AGGREGATOR.attain());
  }

  /**
   * Provides the measured velocity determined via the {@link #sample(WheelObservation) addition} of {@link WheelObservation observations}
   * @return Measured velocity, calculated by the delta between the most recent positions
   */
  public Twist2d getMeasuredVelocity() {
    try {
      WHEEL_UPDATE_LOCK.readLock().lock();
      return Measured;
    } finally {
      WHEEL_UPDATE_LOCK.readLock().unlock();
    }
  }

  /**
   * Provides the predicted velocity determined via the {@link #sample(ChassisSpeeds) addition} of {@link ChassisSpeeds speeds}
   * @return Predicted velocity, calculated by the most recently provided speeds
   */
  public Twist2d getPredictedVelocity() {
    return Predicted;
  } 
  //-----------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   *
   *
   * <h1>WheelObservation</h1>
   *
   */
  public record WheelObservation(List<List<SwerveModulePosition>> Positions, List<Double> Timestamps, List<Rotation2d> Rotations) {}

  /**
   *
   *
   * <h1>VisionObservation</h1>
   *
   */
  public record VisionObservation(Pose2d Position, Matrix<N3,N1> Deviations, Double Timestamp) {}
}