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
import org.frc5411.lib.utility.Operator;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;

import org.littletonrobotics.junction.AutoLog;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>PhoenixRegister</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class PhoenixRegister extends Thread implements Register<StatusSignal<?>,Article> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial 
  static long serialVersionUID = 55742622883094958L;
  static Integer QUEUE_SIZE = (20);
  static Integer UPDATE_FREQUENCY = (100);

  List<Queue<Double>> TIMESTAMPS;  
  List<Queue<Double>> RESPONSES;
  List<StatusSignal<?>> SIGNALS;

  Map<Long,Consumer<ControlRequest>> CLIENTS;
  Map<Long,ControlRequest> REQUESTS;
  
  ReadWriteLock REQUEST_LOCK;
  ReadWriteLock QUEUE_LOCK;
  ReadWriteLock SIGNAL_LOCK;  

  Operator<Double> DISCRETE_OPERATOR;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  @NonFinal static volatile PhoenixRegister Instance = (null);
  @NonFinal static volatile Article State;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Phoenix Register Constructor.
   */
  private PhoenixRegister() {
    State = new Article();
    TIMESTAMPS = new ArrayList<>();
    RESPONSES = new ArrayList<>();
    SIGNALS = new ArrayList<>();
    CLIENTS = new HashMap<>();
    REQUESTS = new HashMap<>();
    DISCRETE_OPERATOR = new Operator<>(
      Timer::getFPGATimestamp, 
      (Previous, Current) -> Current - Previous, 
      Timer.getFPGATimestamp());
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
  public Queue<Double> register(final @NonNull StatusSignal<?> Signal) {
    final var Buffer = new ArrayDeque<Double>(QUEUE_SIZE);
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
   * timing than the robot-main thread's standard 20 millisecond cycle provided by{@link SubsystemBase#periodic()}, providing better control accuracy.
   * 
   * <p> Note that management of ControlRequests is done concurrently with updating {@link #timestamp() timestamp} and {@link #register(StatusSignal) signal} Queues, so if used
   * in tandem, Request-Client pairs also receive the most 'up-to-date' refreshed signal state as feedback.
   * 
   * @param Device  End-point of the control request, essentially what is being controlled via a request
   * @param Request Specified control demand to the end-point (device)
   * @param Client  End-point consumer of the most-recent control request
   */
  public void register(final ParentDevice Device, final ControlRequest Request, final Consumer<ControlRequest> Client) {
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
   * timing than the robot-main thread's standard 20 millisecond cycle provided by{@link SubsystemBase#periodic()}, providing better control accuracy.
   * 
   * <p>Refreshes the current control request with the next control request, returns immediately if the control request does not exist within the
   * current map of Request-Client pairs or is equivalent to the previous request.
   * 
   * @param Device  End-point of the control request, essentially what is being controlled via a request
   * @param Request Specified control demand to the end-point (device)
   */
  public void update(final ParentDevice Device, final ControlRequest Request) {
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
  public Queue<Double> timestamp() {
    final var Buffer = new ArrayDeque<Double>(QUEUE_SIZE);
    try {
      QUEUE_LOCK.writeLock().lock();
      TIMESTAMPS.add(Buffer);
    } finally {
      QUEUE_LOCK.writeLock().unlock();;
    }
    return Buffer;
  }

  @Override
  public synchronized void run() {
    State.Running = (true);
    synchronized(Instance) {
      while(isAlive() && !isInterrupted()) {
        synchronized(PhoenixRegister.class) {
          if(!SIGNALS.isEmpty()) {
            final StatusCode Status;
            try {
              QUEUE_LOCK.writeLock().lock();
              Status = BaseStatusSignal.waitForAll((2D / UPDATE_FREQUENCY), SIGNALS.toArray(BaseStatusSignal[]::new));
            } finally {
              QUEUE_LOCK.writeLock().unlock();
            }
            try {
              SIGNAL_LOCK.writeLock().lock();
              final var Providers = SIGNALS.iterator();
              final Double Timestamp = Timer.getFPGATimestamp() - SIGNALS
                .stream()
                .mapToDouble((Signal) -> 
                  Signal.getTimestamp().getLatency())
                .average()
                .getAsDouble();
              RESPONSES.forEach((final Queue<Double> Queue) -> {
                synchronized(Queue) {
                  Queue.offer(Providers.next().getValueAsDouble());
                }
              });
              TIMESTAMPS.forEach((final Queue<Double> Queue) ->  {
                synchronized(Queue) {
                  Queue.offer(Timestamp);
                }
              });
              State.Priority = getPriority(); 
              State.Status = Status.value;
              State.Period = DISCRETE_OPERATOR.get(); 
              State.Timestamp = DISCRETE_OPERATOR.getRetained();
              State.Registered = SIGNALS.size(); 
              State.Failed = Status.isError()? 1: 0;
            } finally {
              SIGNAL_LOCK.writeLock().unlock();
            }
            try {
              REQUEST_LOCK.readLock().lock();
              CLIENTS.entrySet().forEach((Entry) -> 
                Entry.getValue().accept(REQUESTS.get(Entry.getKey())));
            } finally {
              REQUEST_LOCK.readLock().unlock();
            }
          }
        }
      }
    }
    State.Running = (false);
  }
  //-----------------------------------------------------------------------[Mutators]--------------------------------------------------------------------------//

  
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
  public Article getReport() {
    try {
      SIGNAL_LOCK.readLock().lock();
      return State;
    } finally {
      SIGNAL_LOCK.readLock().unlock();
    }
  }
}
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
/**
 * <h1>Serializable</h1>
 * 
 * <p>Struct serializable instance of a report
 * 
 * @see SerializableStruct
 * 
 */
@FieldDefaults(level = AccessLevel.PROTECTED)
@AutoLog
@Getter
class Article extends Report {
  //----------------------------------------------------------------------[Fields]-----------------------------------------------------------------------------//
  volatile int Failed = (0);
  volatile int Status = (0);
}