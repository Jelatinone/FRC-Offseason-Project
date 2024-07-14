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
import org.frc5411.lib.nouveau.Register;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.pattern.Component;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

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
public abstract class Camera<Hardware> implements Component<Transform3d> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Descriptor<Hardware> DESCRIPTION;
  ReportAutoLogged STATUS;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Camera Constructor.
   * @param Description Real-world description of the system, contains relevant constants to the operation of the camera
   */
  protected Camera(final Descriptor<Hardware> Description) {
    DESCRIPTION = Objects
      .requireNonNull(Description);
    STATUS = new ReportAutoLogged();
    STATUS.setMeasurements(new Transform3d[] {});
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


  /**
   * Provides the current observation Reported during the last {@link #update(Report)} cycle, which means it may be out-of-date if {@link #update(Report)}
   * has not been called for a significant amount of time.
   * @return Latest observation as an optional
   */
  Optional<Pose3d> getObservation() {
    final var Measurements = getObservations();
    return Measurements.isEmpty()? Optional.empty(): Optional.of(Measurements.get(Measurements.size() - (1)));
  }  

  /**
   * Provides a list of all observation (more specifically the different between the positions, deltas, in most cases) that have occurred from the last
   * {@link #update(Report)} cycle until now. This is most often sourced through a queue from a relevant {@link Register} updated asynchronously
   *  of the main-robot thread.
   * @return Latest list of observations
   * @see Register#register(Object) Measurement queues
   * @throws NullPointerException When {@link Report#getMeasurements() observations} has not been properly initialized
   */
  List<Pose3d> getObservations() {
    return List.of(getReport().getObservations().clone());
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