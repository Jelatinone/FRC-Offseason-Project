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
import edu.wpi.first.util.struct.StructSerializable;

import org.littletonrobotics.junction.AutoLog;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.NonNull;
import lombok.Setter;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * 
 * <h1>Report</h1>
 * 
 * <p>Describes a general structure for a reported measurements of a {@link Component device} which produces measurement data of some type with 
 * a changing-value updated {@link Component#update(Report) periodically}. Any types extending this class should be annotated with the relevant
 * {@link AutoLog}, and {@link Getter} annotations. 
 * 
 * <p>Note that the contents of a given report, it's measurements, timestamps, and connection status are all values that should only be mutated
 * from Component of origin's access.<p>
 * 
 * @implNote Array values, such as {@code Timestamps} and {@code Measurements} should be mutated to values in which the component values of the array
 * are ordered in ascending order from oldest to newest; this ensures the correct values are provided from {@link Component#getMeasurement()}
 * and {@link Component#getTimestamps()} by their default implementations.
 * 
 * @see StructSerializable
 */
@FieldDefaults(level = AccessLevel.PROTECTED)
@Getter
@Setter
public abstract class Report<@NonNull Measurement extends StructSerializable> implements Cloneable {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile boolean Connected = (false);

  volatile double[] Timestamps = {};

  volatile Measurement @NonNull[] Measurements; // <---- This property must be set at downstream implementations of Component or ClassCastException is thrown!
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Shorthand for providing an empty instance of a report, with no relevant data stored inside.
   * @param <Measured> Type of the empty report, does not need to be specified in most cases
   * @return Empty report object
   */
  public static <@NonNull Measured extends StructSerializable> Report<Measured> empty() {
    return new Report<>() {};
  }

  /**
   * Creates and returns an entirely new instance of this Report object, which does not retain any of the relevant information
   * of the original instance. 
   * @return new instance of this Report type with no stored data
   */
  public Report<Measurement> clone() {
    return Report.empty();
  }

}
