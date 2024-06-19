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
import edu.wpi.first.util.struct.StructSerializable;

import org.littletonrobotics.junction.AutoLog;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * 
 * <h1>Report</h1>
 * 
 * <p>
 * 
 * @see StructSerializable
 */
@FieldDefaults(level = AccessLevel.PROTECTED)
@Getter
@AutoLog
public class Report {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile int Registered = (0);
  volatile int Priority = (0);
  volatile int Failed = (0);
  volatile int Status = (0);  
  volatile double Period = (-1D);  
  volatile double Average = (-1D);
  volatile double Timestamp = (-1D);
  volatile boolean Running = (false);
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Shorthand for providing an empty instance of a report, with no relevant data stored inside.
   * @return Empty report object
   */
  public static Report empty() {
    return new Report();
  }
}
