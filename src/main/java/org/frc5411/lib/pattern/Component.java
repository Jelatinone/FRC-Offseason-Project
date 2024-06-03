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
package org.frc5411.lib.pattern;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.nouveau.Register;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.struct.StructSerializable;

import org.littletonrobotics.junction.AutoLog;

import java.io.Closeable;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.stream.DoubleStream;

import lombok.NonNull;
import net.bytebuddy.utility.nullability.MaybeNull;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Component</h1>
 * 
 * <p>Describes any given mechanical system that one, can periodically produce a measurement, and two, can periodically produce a report object, which
 * contains the latest measurement, and all the measurements that have occurred between calls to periodic,and lastly, close held resources, such as 
 * motor controllers or encoders. This provides a base implementation of essentially any mechanical system that involves electrical components that produce
 * values.
 * 
 * @see Descriptor
 * @see Report
 * 
 * @author Cody Washington
 */
public interface Component<@NonNull Measured extends StructSerializable> extends Closeable, Sendable {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Generates this Component instance from a relevant descriptor type. Descriptor contains the relevant mechanical constant information to 
   * specify a unique component instance.
   * @param Descriptor Container of a mechanical system's relevant mechanical constants.
   * @return
   */
  static <@NonNull Measured extends StructSerializable> Component<Measured> from(final Descriptor<Component<Measured>> Descriptor) {
    return Descriptor.complete();
  }

  /**
   * Updates the Report of measurements to the most recent measurement data from hardware and {@link Register#register(Object) queue} sources
   * @param Report Loggable source of information, which is automatically logged with the {@link AutoLog} annotation
   * @see Register
   */
  void update(final Report<Measured> Record);


  /**
   * Clones this component instance, throws an exception when this method is called because Component instances are always unique
   * @return                            Nothing, an error is always thrown
   * @throws CloneNotSupportedException When the method is called, because a singleton may only permit a single instance
   */
  public default Component<Measured> clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException();
  }

  /**
   * Performs any necessary logic that this device may need with each update, such as maintaining the position of itself using a controller, or 
   * updating relevant internal values.
   */
  default void periodic() {}

  /**
   * Closes this instance immediately and performs locking-operations to ensure the complete closure of all hardware references. This renders any
   * references to this instance unusable, and should essentially only be done when robot-code has finished operations.
   * @throws IOException When a fatal exception has occurred during an input or output operation.
   */
  default void close() throws IOException {}

  /**
   * Initializes this component as a sendable object over {@link NetworkTable NetworkTables}, meaning the relevant values from {@link #update(Report)}
   * can be published to different dashboards for ease-of-access.
   */
  default void initSendable(final SendableBuilder Builder) {}

  /**
   * Provides the real-world description of the component, essentially an object makeup of the system's mechanical constants
   * @return Descriptor of this component, null by default
   */
  default @MaybeNull Descriptor<Component<Measured>> getDescriptor() {
    return Descriptor.empty();
  }

  /**
   * Provides a full {@link Report} of the measurements of this Component from the last {@link #update(Report)} cycle until now. If {@link #update(Report)}
   * has not been called for a significant amount of time, information may be stale, or out of date.
   * @return Report of measurements, by default an Empty report.
   */
  default Report<Measured> getReport() {
    return Report.empty();
  }  

  /**
   * Provides the current status of connection to this instance's real-world hardware, if this robot is being simulated, then this should always be 
   * true, unless running Unit Tests.
   * @return Status of connection to hardware
   */
  default Boolean getConnection() {
    return getReport().isConnected();
  }

  /**
   * Provides a list of all the timestamps at which {@link #getMeasurements() measurements} have been Reported during the last {@link #update(Report)}
   * cycle until now. This is most often sourced through a queue from a relevant {@link Register} updated asynchronously of the main-robot thread.
   * @return Latest list of measurement timestamps
   * @see {@link Register#timestamp() timestamp queues}
   */
  default List<Double> getTimestamps() {
    return DoubleStream.of(getReport().getTimestamps()).boxed().toList();
  }  

  /**
   * Provides the current measurement Reported during the last {@link #update(Report)} cycle, which means it may be out-of-date if {@link #update(Report)}
   * has not been called for a significant amount of time.
   * @return Latest Measurement 
   */
  default Optional<Measured> getMeasurement() {
    final var Measured = getMeasurements();
    return Optional.ofNullable(Measured.get(Measured.size() - (1)));
  }  

  /**
   * Provides a list of all measurement (more specifically the different between the positions, deltas, in most cases) that have occurred from the last
   * {@link #update(Report)} cycle until now. This is most often sourced through a queue from a relevant {@link Register} updated asynchronously
   *  of the main-robot thread.
   * @return Latest list of measurements
   * @see {@link Register#register(Object) registering queues}
   */
  default List<Measured> getMeasurements() {
    return List.of(getReport().getMeasurements());
  }
}
