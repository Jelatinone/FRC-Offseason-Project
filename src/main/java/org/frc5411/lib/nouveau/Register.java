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
import org.frc5411.lib.schema.Registrable;
import org.frc5411.lib.schema.Singleton;
import org.frc5411.lib.utility.MathUtilities;

import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.locks.ReadWriteLock;

import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Register</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public interface Register<@NonNull Source, @NonNull Serial extends Report> extends Runnable, Singleton<Register<Source,Serial>> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Double STANDARD_FREQUENCY_HERTZ = (250d);
  Integer STARTING_THREAD_PRIORITY = (1);
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * <p>Runs the main-loop of the underlying thread, updating {@link DoubleCircularBuffer buffers} mapped to signals that have been 
   * {@link #register(Registrable) registered}, and updating {@link DoubleCircularBuffer buffers} from the current system {@link Timer#getFPGATimeStamp timestamp}.
   * Additionally, synchronization operations should occur to ensure multi-threading safety through the use of {@link #getSignalLock()  signal locks}.
   * 
   * <p>Should only be called once via {@link #start()}, as the method contains an internal while-loop that continues as long as the Reporter is alive, and
   * {@link #close()} has not yet been called.<p>
   */
  void run();

  /**
   * <p>Makes a call to {@link #run()} and begins updating values appropriately. Ideally, calls to this method are only made once, but the method should contain
   * logic to prevent multiple calls to {@link #run()} concurrently. 
   * <p>However, this method should be capable of being used repeatedly with multiple calls to {@link #start()} and {@link #stop()}
   */
  void start();

  /**
   * Waits for the provided amount of time and attempts to join this thread into the calling thread. Should provide logic
   * to ensure that it can be used repeatedly in conjunction with {@link #start()}.
   * @param Timeout Milliseconds to wait before joining the thread
   */
  void stop(final Long Timeout);

  /**
   * Waits for 'zero' milliseconds and attempts to join this thread into the calling thread. Should provide logic
   * to ensure that it can be used repeatedly in conjunction with {@link #start()}.
   */
  default void stop() {
    stop((0L));
  }

  /**
   * Adds a signal to the collection of signals, these signals act as a source for associated {@link Number numerical} values which are provided back via a
   * Buffer, where the most recently provided element is the last element in the buffer. 
   * 
   * <p> Note that this is done concurrently with updating {@link #timestamp()} buffers, so each update-cycle of {@link #run()} creates a pair of
   * signal-values and timestamps that can be used to better interpolate values as opposed to a standard 20 millisecond cycle provided by 
   * {@link SubsystemBase#periodic()}, which provides lower accuracy.
   * 
   * @param Signal Supplier of Numerical values which can be parsed as a double.
   * @return Buffer, should be retained and used to collect values periodically. 
   * @see MathUtilities#from(DoubleCircularBuffer)
   */
  DoubleCircularBuffer register(final Source Signal);

  /**
   * Creates a new buffer of standard size, and adds it to the collection of timestamp buffers. Each timestamp buffer contains the timestamp from a 
   * {@link Timer#getFPGATimeStamp timestamp} of a {@link #run() run-cycle}, where the last element in the buffer is the most recent timestamp.
   * 
   * <p> Note that this is done concurrently with updating {@link #timestamp()} buffers, so each update-cycle of {@link #run()} creates a pair of
   * signal-values and timestamps that can be used to better interpolate values as opposed to a standard 20 millisecond cycle provided by 
   * {@link SubsystemBase#periodic()}, which provides lower accuracy.
   * 
   * @return Buffer, should be retained and used to collect values periodically. 
   * @see MathUtilities#from(DoubleCircularBuffer)
   */
  DoubleCircularBuffer timestamp(); 
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Provides the current state of this Thread, which can be serialized as a struct value and sent over the network.
   * @return Current state report
   */
  @SuppressWarnings("unchecked")
  default Serial getReport() {
    return (Serial) Report.empty();
  }

  /**
   * Provides the serializable instance of the most recent state provided by {@link #getState()} 
   * @return Current state, as a struct instance
   */
  Struct<Serial> getStruct();

  /**
   * Provides the lock responsible for locking state-update operations and signal and timestamp update operations.
   * @return Lock of signal buffers and timestamp buffers operations
   */
  ReadWriteLock getBufferLock();

  /**
   * Provides the lock responsible for locking {@link #register(Object) registration}, and wait operations
   * @return Lock of registration and wait operations
   */
  ReadWriteLock getSignalLock();
}
