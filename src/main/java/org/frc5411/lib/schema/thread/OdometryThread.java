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
package org.frc5411.lib.schema.thread;
//-------------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//
import org.frc5411.lib.schema.Singleton;

import java.util.Queue;
import java.util.concurrent.locks.Lock;
//------------------------------------------------------------------------[Declaration]------------------------------------------------------------------------//
/**
 *
 *
 * <p>Provides an interface for asynchronously reading high-frequency measurements to a set of queues.<p>
 * 
 * @author Cody Washington
 * 
 * @see CTREOdometryThread
 * @see REVOdometryThread
 * 
 */
@Deprecated(forRemoval = (true), since = ("2024-1.22"))
public sealed interface OdometryThread<Registrable> extends Runnable, Singleton<OdometryThread<Registrable>> permits CTREOdometryThread, REVOdometryThread {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Double STANDARD_FREQUENCY = (250d);
  Integer STANDARD_QUEUE_SIZE = (5);
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Registers a new signal updated at a frequency with the frequency manager.
   * @param Signal Signal source, which can be queried for new signal values
   * @return The {@link Queue} of signal values
   */
  Queue<Double> register(final Registrable Signal);

  /**
   * Provides the timestamps for the available odometry queues.
   * @return The {@link Queue} of timestamp values
   */
  Queue<Double> timestamp(); 

  /**
   * Offers each relevant queue to the relevant signal value.
   */
  void run();

  /**
   * Starts the thread instance of this method, should only be run once.
   */
  void start();
  //---------------------------------------------------------------------[Mutators]----------------------------------------------------------------------------//

  /**
   * Mutates the current frequency of updating the odometry
   * @param Frequency Frequency of odometry updates in Hertz
   */
  void set(final Double Frequency);


  /**
   * Mutates the current state of the thread to be enabled or disabled, note that this has different behavior from {@link #close}, 
   * which stops this instance without the ability to re-enable it.
   * @param Enabled If this Thread is enabled or not
   */
  void set(final Boolean Enabled);
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Provides the lock member-variable of this thread used during it's {@link #run() runnable} operations.
   * @return Synchronization lock of this subsystem
   */
  Lock getLock();

  /**
   * Provides the frequency of this thread
   * @return Frequency as a double
   */
  Double getFrequency();
}