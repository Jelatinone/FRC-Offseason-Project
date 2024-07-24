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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Notifier;

import java.io.Serial;
import java.util.List;
import java.util.Queue;
import java.util.Vector;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>IdentityRegister</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public non-sealed class IdentityRegister<Identity> implements Register<Supplier<Identity>, Identity> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial 
  static long serialVersionUID = 1044457619661863260L;
  static Integer QUEUE_SIZE = (20);
  static Integer UPDATE_FREQUENCY = (250);

  List<Queue<Double>> TIMESTAMPS;  
  List<Queue<Identity>> RESPONSES;
  List<Supplier<Identity>> SIGNALS;

  MedianFilter PEAK_REMOVER;
  LinearFilter LOW_PASS;
  
  ReadWriteLock QUEUE_LOCK;
  ReadWriteLock SIGNAL_LOCK;  

  Notifier CALLBACK;
  Aggregator<Double> DISCRETE_AGGREGATOR;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  @NonFinal volatile ReportAutoLogged State;
  @NonFinal volatile Integer Frequency;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Phoenix Register Constructor.
   */
  public IdentityRegister() {
    State = new ReportAutoLogged();
    TIMESTAMPS = new Vector<>();
    RESPONSES = new Vector<>();
    SIGNALS = new Vector<>();
    PEAK_REMOVER = new MedianFilter((3));
    LOW_PASS = LinearFilter.movingAverage((50));
    DISCRETE_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1E6D, 
      (Previous, Current) -> Current - Previous);
    QUEUE_LOCK = new ReentrantReadWriteLock((true));
    SIGNAL_LOCK = new ReentrantReadWriteLock((true));
    CALLBACK = new Notifier(this);
    CALLBACK.setName(getClass().getSimpleName());
    Frequency = UPDATE_FREQUENCY;
    start();
  }

  //-----------------------------------------------------------------------[Methods]---------------------------------------------------------------------------//
  @Override
  public synchronized Queue<Identity> register(final @NonNull Supplier<Identity> Signal) {
    final var Buffer = new ArrayBlockingQueue<Identity>(QUEUE_SIZE);
    try {
      QUEUE_LOCK.writeLock().lock();
      SIGNALS
        .add(Signal);
      RESPONSES
        .add(Buffer);
      return Buffer;  
    } finally {
      QUEUE_LOCK.writeLock().unlock();
    }
  }

  @Override
  public synchronized Queue<Double> timestamp() {
    final var Buffer = new ArrayBlockingQueue<Double>(QUEUE_SIZE);
    try {
      QUEUE_LOCK.writeLock().lock();
      TIMESTAMPS
        .add(Buffer);
      return Buffer;  
    } finally {
      QUEUE_LOCK.writeLock().unlock();
    }
  }

  @Override
  public synchronized void close() {
    halt();
    try {
      SIGNAL_LOCK.writeLock().lock();
      QUEUE_LOCK.writeLock().lock();     
      synchronized(PhoenixRegister.class) {
        TIMESTAMPS
          .forEach(Queue::clear);
        TIMESTAMPS.clear();
        RESPONSES
          .forEach(Queue::clear);
        RESPONSES.clear();
        SIGNALS.clear();
        State = (null);
      }        
    } finally {
      SIGNAL_LOCK.writeLock().unlock();
      QUEUE_LOCK.writeLock().unlock();      
    }
  }

  @Override
  public final PhoenixRegister clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(
      String.format(
        ("%s Instances Cannot Be Cloned"), 
        getClass()
          .getSimpleName()));
  }  
  
  @Override
  public synchronized void halt(final long Timeout) {
    try {
      Thread.sleep(Timeout);
    } catch (final InterruptedException Ignored) {}
    CALLBACK.close();
  }

  @Override
  public synchronized void start() {
    CALLBACK
      .startPeriodic(1D / Frequency);
  }

  @Override
  public synchronized void run() {
    State.setRunning((true));
    if(!SIGNALS.isEmpty()) {
      try {
        SIGNAL_LOCK.readLock().lock();
        final var Providers = SIGNALS.iterator();
        final var Timestamp = HALUtil.getFPGATime() / 1E6D;
        RESPONSES.forEach((Queue) -> 
          Queue
            .offer(Providers.next().get())
        );
        TIMESTAMPS.forEach((Queue) -> 
          Queue
            .offer(Timestamp)
        );
      } finally {
        SIGNAL_LOCK.readLock().unlock();
        State
          .setRunning((false));
      }
    }
    State.setPeriod(DISCRETE_AGGREGATOR.aggregate());
    State.setAverage(LOW_PASS.calculate(PEAK_REMOVER.calculate(DISCRETE_AGGREGATOR.getAggregated())));
    State.setTimestamp(DISCRETE_AGGREGATOR.getRetained());
    State.setPriority(Thread.currentThread().getPriority());
    State.setRegistered(SIGNALS.size());       
  }
  //---------------------------------------------------------------------[Mutators]----------------------------------------------------------------------------//
  /**
   * Mutates the frequency (hz) at which the internal notifier object of this instance operates at. 
   * @implNote The frequency supplied is clamped according to the implementation of {@link MathUtil#clamp(int,int,int)}, where the minimum (low) value is 1 Hertz, 
   * and the maximum is {@link Register#MAXIMUM_FREQUENCY_HERTZ}
   * @param Frequency Value in hertz, at which this Register should run
   */
  public synchronized void setFrequency(final Integer Frequency) {
    this.Frequency = MathUtil.clamp(
      Frequency, 
      (1), 
      MAXIMUM_FREQUENCY_HERTZ);
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  @Override
  public ReadWriteLock getQueueLock() {
    return QUEUE_LOCK;
  }

  @Override
  public ReadWriteLock getSignalLock() {
    return SIGNAL_LOCK;
  }

  @Override
  public Integer getFrequency() {
    return UPDATE_FREQUENCY;
  }

  @Override
  public ReportAutoLogged getReport() {
    try {
      SIGNAL_LOCK.readLock().lock();
      return State;
    } finally {
      SIGNAL_LOCK.readLock().unlock();
    }
  }
}