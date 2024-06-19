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
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.ArrayDeque;
import java.util.Vector;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>PhoenixRegister</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class PhoenixRegister extends Thread implements Register<StatusSignal<Number>,Report> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial 
  static long serialVersionUID = 55742622883094958L;
  static Integer QUEUE_SIZE = (20);
  static Integer UPDATE_FREQUENCY = (100);

  List<Queue<Double>> TIMESTAMPS;  
  List<Queue<Optional<Number>>> RESPONSES;
  List<StatusSignal<Number>> SIGNALS;

  Map<Long,Consumer<ControlRequest>> CLIENTS;
  Map<Long,ControlRequest> REQUESTS;

  MedianFilter PEAK_REMOVER;
  LinearFilter LOW_PASS;
  
  ReadWriteLock REQUEST_LOCK;
  ReadWriteLock QUEUE_LOCK;
  ReadWriteLock SIGNAL_LOCK;  

  Aggregator<Double> DISCRETE_AGGREGATOR;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  static volatile PhoenixRegister Instance = (null);
  static volatile ReportAutoLogged State;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Phoenix Register Constructor.
   */
  private PhoenixRegister() {
    State = new ReportAutoLogged();
    TIMESTAMPS = new Vector<>();
    RESPONSES = new Vector<>();
    SIGNALS = new Vector<>();
    CLIENTS = new HashMap<>();
    REQUESTS = new HashMap<>();
    PEAK_REMOVER = new MedianFilter((3));
    LOW_PASS = LinearFilter.movingAverage((50));
    DISCRETE_AGGREGATOR = new Aggregator<>(
      () -> HALUtil.getFPGATime() / 1e6, 
      (Previous, Current) -> Current - Previous);
    REQUEST_LOCK = new ReentrantReadWriteLock((true));
    QUEUE_LOCK = new ReentrantReadWriteLock((true));
    SIGNAL_LOCK = new ReentrantReadWriteLock((true));  
    setDaemon((true));
    setName(getClass().getCanonicalName());
    start();
  }
  //-----------------------------------------------------------------------[Methods]---------------------------------------------------------------------------//
  @Serial
  @Override
  public synchronized PhoenixRegister readResolve() {
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
    halt();
    REQUEST_LOCK.writeLock().lock();
    QUEUE_LOCK.writeLock().lock();
    synchronized(PhoenixRegister.class) {
      TIMESTAMPS.forEach(Queue::clear);
      TIMESTAMPS.clear();
      RESPONSES.forEach(Queue::clear);
      RESPONSES.clear();
      SIGNALS.clear();
      CLIENTS.clear();
      REQUESTS.clear();
      State = (null);
      Instance = (null);
      REQUEST_LOCK.writeLock().unlock();
      QUEUE_LOCK.writeLock().unlock();
    }
  }

  @Override
  public final PhoenixRegister clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getCanonicalName()));
  }

  @Override
  public synchronized void halt(final Long Timeout) {
    try {
      join(Timeout);
    } catch(final InterruptedException Exception) {
      currentThread().interrupt();
    }
  }

  @Override
  public synchronized Queue<Optional<Number>> register(final @NonNull StatusSignal<Number> Signal) {
    final var Buffer = new ArrayDeque<Optional<Number>>(QUEUE_SIZE);
    try {
      QUEUE_LOCK.writeLock().lock();
      Signal.setUpdateFrequency(UPDATE_FREQUENCY);
      SIGNALS.add(Signal);
      RESPONSES.add(Buffer);
    } finally {
      QUEUE_LOCK.writeLock().unlock();
    }
    return Buffer;
  }

  /**
   * Adds a Request-Client pair to be managed by the register; client endpoints receive the latest state of the request via the register on faster
   * timing than the robot-main thread's standard 20-millisecond cycle provided by{@link SubsystemBase#periodic()}, providing better control accuracy.
   * 
   * <p> Note that management of ControlRequests is done concurrently with updating {@link #timestamp() timestamp} and {@link #register(StatusSignal) signal} Queues, so if used
   * in tandem, Request-Client pairs also receive the most 'up-to-date' refreshed signal state as feedback.
   * 
   * @param Device  End-point of the control request, essentially what is being controlled via a request
   * @param Request Specified control demand to the end-point (device)
   * @param Client  End-point consumer of the most-recent control request
   */
  public synchronized void register(final ParentDevice Device, final ControlRequest Request, final Consumer<ControlRequest> Client) {
    final var Hash = Device.getDeviceHash();
    try {
      REQUEST_LOCK.writeLock().unlock();
      REQUESTS.put(Hash, Request);
      CLIENTS.put(Hash, Client);
    } finally {
      REQUEST_LOCK.writeLock().unlock();
    }
  }

  /**
   * Refreshes a Request-Client pair to be managed by the register; client endpoints receive the latest state of the request via the register on faster
   * timing than the robot-main thread's standard 20-millisecond cycle provided by{@link SubsystemBase#periodic()}, providing better control accuracy.
   * 
   * <p>Refreshes the current control request with the next control request, returns immediately if the control request does not exist within the
   * current map of Request-Client pairs or is equivalent to the previous request.
   * 
   * @param Device  End-point of the control request, essentially what is being controlled via a request
   * @param Request Specified control demand to the end-point (device)
   */
  public synchronized void update(final ParentDevice Device, final ControlRequest Request) {
    final var Hash = Device.getDeviceHash();
    if(REQUESTS.get(Hash) == Request) {
      return;
    }
    try {
      REQUEST_LOCK.writeLock().unlock();
      REQUESTS.put(Hash, Request);
    } finally {
      REQUEST_LOCK.writeLock().unlock();
    }
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
  public synchronized void run() {
    synchronized(Instance) {
      while(isAlive() && !isInterrupted()) {
        State.Running = (true);
        synchronized(PhoenixRegister.class) {
          if(!SIGNALS.isEmpty()) {
            final StatusCode Status;
            try {
              QUEUE_LOCK.writeLock().lock();
              Status = BaseStatusSignal.waitForAll(
                2D / UPDATE_FREQUENCY, 
                SIGNALS.toArray(BaseStatusSignal[]::new));
            } finally {
              QUEUE_LOCK.writeLock().unlock();
            }
            try {
              SIGNAL_LOCK.writeLock().lock();
              final var Providers = SIGNALS.iterator();
              final var Timestamp = DISCRETE_AGGREGATOR.attain() - SIGNALS
                .stream()
                .mapToDouble((Signal) -> Signal.getTimestamp().getLatency())
                .average()
                .orElse((0D));
              RESPONSES.forEach((final Queue<Optional<Number>> Queue) -> {
                synchronized(Queue) {
                  Queue.offer(Optional.ofNullable(Providers.next().getValue()));
                }
              });
              TIMESTAMPS.forEach((final Queue<Double> Queue) ->  {
                synchronized(Queue) {
                  Queue.offer(Timestamp);
                }
              });
            } finally {
              SIGNAL_LOCK.writeLock().unlock();
            }
            try {
              REQUEST_LOCK.readLock().lock();
              CLIENTS.forEach((Hash, Applicator) -> Applicator.accept(REQUESTS.get(Hash)));
            } finally {
              REQUEST_LOCK.readLock().unlock();
            }
            State.Priority = getPriority(); 
            State.Status = Status.value;
            State.Period = DISCRETE_AGGREGATOR.aggregate(); 
            State.Average = LOW_PASS.calculate(
              PEAK_REMOVER.calculate(DISCRETE_AGGREGATOR.getAggregated()));
            State.Timestamp = DISCRETE_AGGREGATOR.getRetained();
            State.Registered = SIGNALS.size(); 
            State.Failed = Status.isError()? 1: 0;            
          }
        }
        State.Running = (false);
      }
    }
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized PhoenixRegister getInstance() {
    PhoenixRegister Result = Instance;
    if(Instance == (null)) {
      synchronized(PhoenixRegister.class) {
        Result = Instance;
        if(Instance == (null)) {
          Instance = Result = new PhoenixRegister();
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