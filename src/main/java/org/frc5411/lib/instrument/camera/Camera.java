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
package org.frc5411.lib.instrument.camera;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.pattern.Component;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.Objects;
import java.util.stream.Stream;

import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Camera</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(makeFinal = (true), level = AccessLevel.PRIVATE)
public abstract class Camera<Hardware> implements Component<Pose3d> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Descriptor<Hardware> DESCRIPTION;
  Report STATUS;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Camera Constructor.
   * @param Description Real-world description of the system, contains relevant constants to the operation of the camera
   */
  protected Camera(final Descriptor<Hardware> Description) {
    DESCRIPTION = Objects
      .requireNonNull(Description);
    STATUS = new Report();
    STATUS.setMeasurements(new Pose3d[] {});
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public final Camera<Hardware> clone() throws CloneNotSupportedException {
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
      if(getConnection()) {
        STATUS.setMerit(
          Stream.of(STATUS.getObservations())
          .reduce((Current, Next) -> Next)
          .map((Observation) -> 
            Stream.of(Observation)
              .map((Transform) -> 
                Transform.getTranslation().getNorm())
              .min(Double::compareTo)
              .orElse(Double.NaN))
          .orElse(Double.NaN));
      } else {
        STATUS
          .setMerit(Double.POSITIVE_INFINITY);
      }
    }
    /*
     * See the following which references the below issue:
     * https://github.com/Mechanical-Advantage/AdvantageKit/issues/97
     * 
     * Logger.processInputs(
     *  getIdentity(), STATUS);   
     */
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

  @Override
  public String getIdentity() {
    return String.format(
      ("%s-[%s]"),
      getClass().getSimpleName().toUpperCase(),
      DESCRIPTION.Identity.name()
    );
  }
}