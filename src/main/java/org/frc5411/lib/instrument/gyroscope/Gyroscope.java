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
package org.frc5411.lib.instrument.gyroscope;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.pattern.Component;

import edu.wpi.first.math.geometry.Rotation3d;

import org.littletonrobotics.junction.Logger;

import java.util.Objects;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Gyroscope</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(makeFinal = (true), level = AccessLevel.PRIVATE)
public abstract class Gyroscope<Hardware> implements Component<Rotation3d> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Descriptor<Hardware> DESCRIPTION;
  ReportAutoLogged STATUS;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Gyroscope Constructor.
   * @param Description Real-world description of the system, contains relevant constants to the operation of the gyroscope
   */
  protected Gyroscope(final Descriptor<Hardware> Description) {
    DESCRIPTION = Objects.requireNonNull(Description);
    STATUS = new ReportAutoLogged();
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public final Gyroscope<Hardware> clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(
      String.format(
        ("%s Instances Cannot Be Cloned"), 
        getClass()
          .getSimpleName()));
  }

  @Override
  public synchronized void periodic() {
    synchronized(STATUS) {
      update(STATUS);
    }
    Logger.processInputs(
      getIdentity(), STATUS);   
  }
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  @Override
  public Report getReport() {
    return STATUS;
  }

  @Override
  public Descriptor<Hardware> getDescriptor() {
    return DESCRIPTION;
  }
}
