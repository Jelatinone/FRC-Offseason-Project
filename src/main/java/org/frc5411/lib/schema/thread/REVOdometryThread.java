//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.schema.thread;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.wpilibj.Notifier;

import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
//------------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 *
 *
 * <p>Provides an interface for asynchronously reading high-frequency measurements to a set of queues. This version is intended for devices
 * like the SparkMax that require polling rather than a blocking thread. A Notifier thread is used to gather samples with consistent timing.
 * 
 * <p>This file was converted into List and Stream(able) objects (lists) for ease of interfacing with data, from Mechanical Advantage's original
 * implementation found <a href="https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/example_projects/advanced_swerve_drive/src/main/java/frc/robot/subsystems/drive/SparkMaxOdometryThread.java">here</a>.
 * 
 * @see OdometryThread
 * @see Notifier
 * 
 * @author Cody Washington (Rewrite)
 * @author Mechanical Advantage (Original)
 * 
 */
public final class REVOdometryThread implements OdometryThread<Supplier<Number>> {  
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  private static final long serialVersionUID = 84309938899889961L;
  private static final List<Supplier<Number>> SIGNAL_PROVIDERS;
  private static final List<Queue<Double>> TIMESTAMP_QUEUES;
  private static final List<Queue<Double>> SIGNAL_QUEUES;
  private final Lock ODOMETRY_LOCK;
  private final Notifier NOTIFIER;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  private volatile static REVOdometryThread Instance;
  private volatile static Boolean Enabled;
  private volatile static Double Frequency;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * REV Odometry Thread Constructor.
   */
  private REVOdometryThread() {
    ODOMETRY_LOCK = new ReentrantLock((true));
    NOTIFIER = new Notifier(this);
    NOTIFIER.setName(this.getClass().getSimpleName());
    start();
  } static {
    SIGNAL_QUEUES = new ArrayList<>();
    TIMESTAMP_QUEUES = new ArrayList<>();
    SIGNAL_PROVIDERS = new ArrayList<>();
    Instance = (null);
    Enabled = (false);
    Frequency = STANDARD_FREQUENCY;
  }
  //-----------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized Queue<Double> register(final Supplier<Number> Signal) {
    Queue<Double> Queue = new ArrayBlockingQueue<>(STANDARD_QUEUE_SIZE);
    ODOMETRY_LOCK.lock();
    try {
      SIGNAL_PROVIDERS.add(Signal);
      SIGNAL_QUEUES.add(Queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return Queue;
  }

  @Serial
  @Override
  public synchronized REVOdometryThread readResolve() {
    return Instance;
  }

  @Serial
  @Override
  public synchronized void readObject(final ObjectInputStream Stream) throws IOException, ClassNotFoundException {
    Stream.defaultReadObject();
    Instance = (this);
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

  public synchronized void start() {
    if (TIMESTAMP_QUEUES.isEmpty()) {
      NOTIFIER.startPeriodic((1d)/ Frequency);
    }
  }

  @Override
  public synchronized void close() throws IOException {
    ODOMETRY_LOCK.lock();
    synchronized(REVOdometryThread.class) {
      NOTIFIER.stop();  
      SIGNAL_QUEUES.forEach(Queue::clear);
      SIGNAL_QUEUES.clear();
      TIMESTAMP_QUEUES.forEach(Queue::clear);
      TIMESTAMP_QUEUES.clear();
      SIGNAL_PROVIDERS.clear(); 
    }
    ODOMETRY_LOCK.unlock();
  }

  @Override
  public final Object clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(("Singleton Instances Cannot Be Cloned"));
  }

  @SuppressWarnings("SynchronizationOnLocalVariableOrMethodParameter")
  @Override
  public synchronized void run() {
    if(Enabled && !SIGNAL_PROVIDERS.isEmpty()) {
      try {
        ODOMETRY_LOCK.lock();
        try {
          final var Providers = SIGNAL_PROVIDERS.iterator();
          final var Timestamp = Logger.getRealTimestamp() / (1e6);
          SIGNAL_QUEUES.forEach((final Queue<Double> Queue) -> {
            synchronized(Queue) {
              Queue.offer(Providers.next().get().doubleValue());
            }
          });
          TIMESTAMP_QUEUES.forEach((final Queue<Double> Queue) ->  {
            synchronized(Queue) {
              Queue.offer(Timestamp);
            }
          });
        } finally {
          ODOMETRY_LOCK.unlock();
        }
      } catch (final Exception Ignored) {}      
    }
  }
  //---------------------------------------------------------------------[Mutators]------------------------------------------------------------------------//
  
  @Override
  public synchronized void set(final Double Frequency) {
    ODOMETRY_LOCK.lock();
    REVOdometryThread.Frequency = Math.min(Frequency, (1000d));
    NOTIFIER.stop();
    NOTIFIER.startPeriodic(Frequency);
    ODOMETRY_LOCK.unlock();
  }


  @Override
  public synchronized void set(final Boolean Enabled) {
    REVOdometryThread.Enabled = Enabled;
  }
  //---------------------------------------------------------------------[Accessors]--------------------------------------------------------------------------//
  @Override
  public Lock getLock() {
    return ODOMETRY_LOCK;
  }

  @Override
  public Double getFrequency() {
    return Frequency;
  }  

  /**
   * Creates a new instance of the existing utility class
   * @return Utility class's instance
   */
  public static synchronized REVOdometryThread getInstance() {
    var Result = Instance;
    if (Instance == (null)) {
      synchronized (REVOdometryThread.class) {
        if (Instance == (null)) {
          Instance = Result = new REVOdometryThread();
        }
      }
    }
    return Result;
  }
}