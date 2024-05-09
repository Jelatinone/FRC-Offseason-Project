//--------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.schema.thread;
//-------------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
//------------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 *
 *
 * <p>Provides an interface for asynchronously reading high-frequency measurements to a set of queues. This version is intended for Phoenix
 * 6 devices on both the RIO and CANivore buses. When using a CANivore, the thread uses the "waitForAll" blocking method to enable more
 * consistent sampling. This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore time synchronization.
 * 
 * <p>This file was converted into List and Stream(able) objects (lists) for ease of interfacing with data, from Mechanical Advantage's original
 * implementation found <a href="https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/example_projects/advanced_swerve_drive/src/main/java/frc/robot/subsystems/drive/PhoenixOdometryThread.java">here</a>.
 * 
 * @see OdometryThread
 * @see Thread
 * 
 * @author Cody Washington (Rewrite)
 * @author Mechanical Advantage (Original)
 * 
 */
public final class CTREOdometryThread extends Thread implements OdometryThread<StatusSignal<?>> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  private static final long serialVersionUID = 55742622883094958L;
  private static final List<StatusSignal<?>> SIGNAL_PROVIDERS;
  private static final List<Queue<Double>> TIMESTAMP_QUEUES;  
  private static final List<Queue<Double>> SIGNAL_QUEUES;
  private static final Lock SIGNALS_LOCK;
  private static final Lock ODOMETRY_LOCK;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  private volatile static CTREOdometryThread Instance;
  private volatile static Boolean Enabled;
  private volatile static Double Frequency;
  private volatile static Boolean Flexible;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Phoenix Odometry Thread Constructor.
   */
  private CTREOdometryThread() {
    setName(this.getClass().getSimpleName());
    setDaemon((true));
    start();
  } static {
    SIGNAL_PROVIDERS = new ArrayList<>();
    SIGNAL_QUEUES = new ArrayList<>();
    TIMESTAMP_QUEUES = new ArrayList<>();
    SIGNALS_LOCK = new ReentrantLock();
    ODOMETRY_LOCK = new ReentrantLock((true));
    Frequency = STANDARD_FREQUENCY;
    Instance = (null);
    Flexible = (false);
    Enabled = (false);
  }
  //-----------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized void start() {
    if (TIMESTAMP_QUEUES.isEmpty()) {
      super.start();
    }
  }

  @Serial
  @Override
  public synchronized CTREOdometryThread readResolve() {
    return Instance;
  }

  @Serial
  @Override
  public synchronized void readObject(final ObjectInputStream Stream) throws IOException, ClassNotFoundException {
    Stream.defaultReadObject();
    Instance = (this);
  }
  
  @Override
  public synchronized Queue<Double> register(final StatusSignal<?> Signal) {
    Queue<Double> Queue = new ArrayBlockingQueue<>(STANDARD_QUEUE_SIZE);
    SIGNALS_LOCK.lock();
    ODOMETRY_LOCK.lock();
    try {
      SIGNAL_PROVIDERS.add(Signal);
      SIGNAL_QUEUES.add(Queue);
    } finally {
      SIGNALS_LOCK.unlock();
      ODOMETRY_LOCK.unlock();
    }
    return Queue;
  }

  @Override
  public synchronized void close() throws IOException {
    SIGNALS_LOCK.lock();
    ODOMETRY_LOCK.lock();
    synchronized(CTREOdometryThread.class) {
      this.interrupt();
      SIGNAL_QUEUES.forEach(Queue::clear);
      SIGNAL_QUEUES.clear();
      TIMESTAMP_QUEUES.forEach(Queue::clear);
      TIMESTAMP_QUEUES.clear();
      SIGNAL_PROVIDERS.clear();      
    }
    ODOMETRY_LOCK.unlock();
    SIGNALS_LOCK.unlock();
  }

  @Override
  public synchronized Queue<Double> timestamp() {
    Queue<Double> Queue = new ArrayBlockingQueue<>(STANDARD_QUEUE_SIZE);
    ODOMETRY_LOCK.lock();
    try {
      TIMESTAMP_QUEUES.add(Queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return Queue;
  }

  @Override
  public final Object clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(("Singleton Instances Cannot Be Cloned"));
  }

  @SuppressWarnings({"BusyWait", "SynchronizationOnLocalVariableOrMethodParameter"})
  @Override
  public synchronized void run() {
    while (this.isAlive()) {
      try {
        SIGNALS_LOCK.lock();
        try {
          final var Signals = SIGNAL_PROVIDERS.toArray(StatusSignal[]::new);
          if (Flexible) {
            BaseStatusSignal.waitForAll((2d) / Frequency, Signals);
          } else {
            Thread.sleep((long) ((1000d) / Frequency.longValue()));
            if (Signals != null) {
              BaseStatusSignal.refreshAll(Signals);
            }
          }
        } catch (final InterruptedException Ignored) {} finally {
          SIGNALS_LOCK.unlock();
        }
        if(Enabled && !SIGNAL_PROVIDERS.isEmpty()) {
          ODOMETRY_LOCK.lock();
          try {
            final var Providers = SIGNAL_PROVIDERS.iterator();
            final var Timestamp = new AtomicReference<>(Logger.getRealTimestamp() / (1e6));
            Timestamp.accumulateAndGet(SIGNAL_PROVIDERS.stream().mapToDouble(
              (Signal) -> Signal.getTimestamp().getLatency()).average().getAsDouble(), (a, b) -> a -= b);
            SIGNAL_QUEUES.forEach((final Queue<Double> Queue) -> {
              synchronized(Queue) {
                Queue.offer(Providers.next().getValueAsDouble());
              }
            });
            TIMESTAMP_QUEUES.forEach((final Queue<Double> Queue) ->  {
              synchronized(Queue) {
                Queue.offer(Timestamp.get());
              }
            });
          } finally {
            ODOMETRY_LOCK.unlock();
          }          
        } if(isInterrupted()) {
          break;
        }           
      } catch (final Exception Ignored) {}         
    }
  }  
  //---------------------------------------------------------------------[Mutators]------------------------------------------------------------------------//
  /**
   * Mutates the current status of the can bus to determine if it supports flexible data rates.
   * @param Flexible If the CAN bus of devices is flexible
   */
  public synchronized void setFlexibility(final Boolean Flexible) {
    CTREOdometryThread.Flexible = Flexible;
  }

  @Override
  public synchronized void set(final Boolean Enabled) {
    CTREOdometryThread.Enabled = Enabled;
  }

  @Override
  public synchronized void set(final Double Frequency) {
    CTREOdometryThread.Frequency = Math.min(Frequency, (1000d));
  }
  //---------------------------------------------------------------------[Accessors]-----------------------------------------------------------------------//
  @Override
  public Lock getLock() {
    return ODOMETRY_LOCK;
  }

  @Override
  public Double getFrequency() {
    return Frequency;
  }  

  /**
   * Creates a new instance of the existing utility class, if it already exists, retrieve the same instance.
   * @return Utility class's instance
   */
  public static synchronized CTREOdometryThread getInstance() {
    var Result = Instance;
    if (Instance == (null)) {
      synchronized (CTREOdometryThread.class) {
        if (Instance == (null)) {
          Instance = Result = new CTREOdometryThread();
        }
      }
    }
    return Result;
  }
}