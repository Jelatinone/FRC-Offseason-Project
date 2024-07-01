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

import org.frc5411.robot2024.subsystems.drivebase.DrivebaseSubsystem;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.Notifier;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
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
public final class Manager implements Singleton<Manager>, Runnable {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  static long serialVersionUID = 2389697764281159320L;
  static Integer UPDATE_FREQUENCY = (100);
  static Integer QUEUE_SIZE = (20);
  static Double BUFFER_SIZE = (2D);
  static Matrix<N2,N1> STATE_STANDARD_DEVIATIONS = VecBuilder.fill((1D),(1D));
  static Matrix<N2,N1> MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill((1D),(1D));
  static Aggregator<Double> DISCRETE_AGGREGATOR;

  ReadWriteLock WHEEL_UPDATE_LOCK;
  ReadWriteLock VISION_UPDATE_LOCK;  

  Queue<WheelObservation> WHEEL_UPDATE_QUEUE;
  Queue<VisionObservation> VISION_UPDATE_QUEUE;

  TimeInterpolatableBuffer<Pose2d> VEHICLE_ODOMETRY;
  TimeInterpolatableBuffer<Translation2d> FIELD_ODOMETRY;

  SwerveDriveKinematics KINEMATICS;
  SwerveDriveOdometry ODOMETRY;  
  ExtendedKalmanFilter<N2,N2,N2> FILTER;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile Manager Instance;
  static volatile SwerveDriveWheelPositions Positions;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Manager Constructor.
   */
  private Manager() {
    WHEEL_UPDATE_LOCK = new ReentrantReadWriteLock((true));
    VISION_UPDATE_LOCK = new ReentrantReadWriteLock((true));
    WHEEL_UPDATE_QUEUE = new ArrayDeque<>(QUEUE_SIZE);
    VISION_UPDATE_QUEUE = new ArrayDeque<>(QUEUE_SIZE);
    VEHICLE_ODOMETRY = TimeInterpolatableBuffer.createBuffer(BUFFER_SIZE);
    FIELD_ODOMETRY = TimeInterpolatableBuffer.createBuffer(BUFFER_SIZE);
    FILTER = new ExtendedKalmanFilter<>(
      Nat.N2(),
      Nat.N2(),
      Nat.N2(),
      (Input, Output) -> Output,
      (Input, Output) -> Input,
      STATE_STANDARD_DEVIATIONS,
      MEASUREMENT_STANDARD_DEVIATIONS,
      1D / UPDATE_FREQUENCY);
    KINEMATICS = (null);
    ODOMETRY = (null);
    Positions = new SwerveDriveWheelPositions(DrivebaseSubsystem.getInstance().getModulePositions());
  } static {
    //<--- Fetch All Managed Subsystems --->
    DrivebaseSubsystem.getInstance();

    //<--- Construct static fields --->
    DISCRETE_AGGREGATOR = new Aggregator<>(
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
   * Performs queued robot-wide actions at a higher frequency than a subsystem instance; such as robot-wide 
   * odometry or specific sensor updates which require higher update frequencies, but should not be contained
   * within their own {@link Notifier} or separate {@link Thread} instance(s).
   */
  public synchronized void run() {
    if(Instance != null) {
      synchronized(Manager.class) {
        DISCRETE_AGGREGATOR.aggregate();
        try {
          WHEEL_UPDATE_LOCK.readLock().lock();
          WHEEL_UPDATE_QUEUE.forEach((final WheelObservation Observation) -> {

          });
          WHEEL_UPDATE_QUEUE.clear();
        } finally {
          WHEEL_UPDATE_LOCK.readLock().unlock();
        }
        try {
          VISION_UPDATE_LOCK.readLock().lock();
          VISION_UPDATE_QUEUE.forEach((final VisionObservation Observation) -> {

          });
          VISION_UPDATE_QUEUE.clear();
        } finally {
          VISION_UPDATE_LOCK.readLock().unlock();
        }
      }      
    }
  }

  @Override
  public Manager clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getSimpleName()));
  }

  /**
   * <p>Adds a new WheelObservation instance, containing the necessary information to calculate accurate wheel odometry values when the {@link #run()} method 
   * is called, to the wheel observation queue.
   * <p>Note that because of the nature of a queue collection, and that the added elements are processed asynchronously, this method can handle dozens of 
   * repeat calls (such as during Subsystem#periodic()) to calculate far more accurate odometry values
   * @param Observation Wheel observation to add to the processing queue
   */
  public synchronized void add(final WheelObservation Observation) {
    try {
      WHEEL_UPDATE_LOCK.writeLock().lock();
      WHEEL_UPDATE_QUEUE.offer(Observation);
    } finally {
      WHEEL_UPDATE_LOCK.writeLock().unlock();
    }
  }  

  /**
   * <p>Adds a new VisionObservation instance, containing the necessary information to calculate accurate vision odometry values when the {@link #run()} method 
   * is called, to the vision observation queue.
   * <p>Note that because of the nature of a queue collection, and that the added elements are processed asynchronously, this method can handle dozens of 
   * repeat calls (such as during Subsystem#periodic()) to calculate far more accurate odometry values
   * @param Observation Vision observation to add to the processing queue
   */
  public synchronized void add(final VisionObservation Observation) {
    try {
      VISION_UPDATE_LOCK.writeLock().lock();
      VISION_UPDATE_QUEUE.offer(Observation);
    } finally {
      VISION_UPDATE_LOCK.writeLock().unlock();
    }
  }
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
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
  } static {
    Robot.add(Instance, 1D / UPDATE_FREQUENCY);
  }
  //-----------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   *
   *
   * <h1>WheelObservation</h1>
   *
   */
  public record WheelObservation(SwerveDriveWheelPositions Position, Optional<Rotation2d> Rotation, Double Timestamp) {}

  /**
   *
   *
   * <h1>VisionObservation</h1>
   *
   */
  public record VisionObservation(Pose2d Position, Matrix<N5,N1> Deviations, Double Timestamp) {}
}