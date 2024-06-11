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
package org.frc5411.lib.nouveau;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.utility.Aggregator;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Notifier;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>StandardRegister</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class StandardRegister implements Register<Supplier<Optional<Number>>, Report>{
  /*
   * TODO:
   *    Optional-Support/Requirement
   */
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial 
  static long serialVersionUID = 84309938899889961L;
  static Integer QUEUE_SIZE = (20);
  static Integer UPDATE_FREQUENCY = (100);

  List<Queue<Double>> TIMESTAMPS;  
  List<Queue<Optional<Number>>> RESPONSES;
  List<Supplier<Optional<Number>>> SIGNALS;

  MedianFilter PEAK_REMOVER;
  LinearFilter LOW_PASS;

  ReadWriteLock QUEUE_LOCK;
  ReadWriteLock SIGNAL_LOCK;  

  Notifier CALLBACK;
  Aggregator<Double> DISCRETE_AGGREGATOR;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile StandardRegister Instance = (null);
  static volatile ReportAutoLogged State;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Standard Register Constructor.
   */
  private StandardRegister() {
    State = new ReportAutoLogged();
    CALLBACK = new Notifier(this);
    TIMESTAMPS = new ArrayList<>();
    RESPONSES = new ArrayList<>();
    SIGNALS = new ArrayList<>();
    PEAK_REMOVER = new MedianFilter((3));
    LOW_PASS = LinearFilter.movingAverage((50));
    DISCRETE_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1e6, 
      (Previous, Current) -> Current - Previous);
    QUEUE_LOCK = new ReentrantReadWriteLock((true));
    SIGNAL_LOCK = new ReentrantReadWriteLock((true));
    CALLBACK.setName(getClass().getCanonicalName());
    start();
  }
  //-----------------------------------------------------------------------[Methods]---------------------------------------------------------------------------//
  @Serial
  @Override
  public synchronized StandardRegister readResolve() {
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
    CALLBACK.close();
    SIGNAL_LOCK.writeLock().lock();
    QUEUE_LOCK.writeLock().lock();
    synchronized(StandardRegister.class) {
      TIMESTAMPS.forEach(Queue::clear);
      TIMESTAMPS.clear();
      RESPONSES.forEach(Queue::clear);
      RESPONSES.clear();
      SIGNALS.clear();
      State = (null);
      Instance = (null);
      SIGNAL_LOCK.writeLock().unlock();
      QUEUE_LOCK.writeLock().unlock();
    }
  }

  @Override
  public final StandardRegister clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getCanonicalName()));
  }

  @Override
  public synchronized void halt(final Long Timeout) {
    try {
      Thread.sleep(Timeout);
    } catch (final InterruptedException Ignored) {}
    CALLBACK.stop();
  }

  @Override
  public synchronized Queue<Optional<Number>> register(final @NonNull Supplier<Optional<Number>> Signal) {
    final var Buffer = new ArrayDeque<Optional<Number>>(QUEUE_SIZE);
    try {
      QUEUE_LOCK.writeLock().lock();
      SIGNALS.add(Signal);
      RESPONSES.add(Buffer);
    } finally {
      QUEUE_LOCK.writeLock().unlock();
    }
    return Buffer;
  }

  @Override
  public synchronized Queue<Double> timestamp() {
    final var Buffer = new ArrayDeque<Double>(QUEUE_SIZE);
    try {
      QUEUE_LOCK.writeLock().lock();
      TIMESTAMPS.add(Buffer);
    } finally {
      QUEUE_LOCK.writeLock().unlock();
    }
    return Buffer;
  }

  @Override
  public synchronized void start() {
    CALLBACK.startPeriodic(1D/UPDATE_FREQUENCY);
  }

  @Override
  public synchronized void run() {
    State.Running = (true);
    if(!SIGNALS.isEmpty()) {
      try {
        SIGNAL_LOCK.readLock().lock();
        final var Providers = SIGNALS.iterator();
        final var Timestamp = HALUtil.getFPGATime() / 1e6;
        RESPONSES.forEach((final Queue<Optional<Number>> Queue) -> {
          synchronized(Queue) {
            Queue.offer(Providers.next().get());
          }
        });
        TIMESTAMPS.forEach((final Queue<Double> Queue) -> {
          synchronized(Queue) {
            Queue.offer(Timestamp);
          }
        });
        State.Period = DISCRETE_AGGREGATOR.aggregate();
        State.Average = LOW_PASS.calculate(
          PEAK_REMOVER.calculate(DISCRETE_AGGREGATOR.getAggregated()));
        State.Timestamp = DISCRETE_AGGREGATOR.getRetained();
        State.Priority = ThreadsJNI.getCurrentThreadPriority();
        State.Registered = SIGNALS.size();
      } finally {
        SIGNAL_LOCK.readLock().unlock();
      }
    }
    State.Running = (false);
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized StandardRegister getInstance() {
    StandardRegister Result = Instance;
    if(Instance == (null)) {
      synchronized(StandardRegister.class) {
        Result = Instance;
        if(Instance == (null)) {
          Instance = Result = new StandardRegister();
        }
      }
    }
    return Result;
  }

  @Override
  public ReadWriteLock getQueueLock() {
    return QUEUE_LOCK;
  }

  @Override
  public ReadWriteLock getSignalLock() {
    return SIGNAL_LOCK;
  }

  @Override
  public Report getReport() {
    try {
      SIGNAL_LOCK.readLock().lock();
      return State;
    } finally {
      SIGNAL_LOCK.readLock().unlock();
    }
  }
}