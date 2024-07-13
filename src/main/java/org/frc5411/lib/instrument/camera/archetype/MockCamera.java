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
package org.frc5411.lib.instrument.camera.archetype;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.instrument.camera.Camera;
import org.frc5411.lib.instrument.camera.Descriptor;

import org.photonvision.simulation.PhotonCameraSim;
import edu.wpi.first.math.geometry.Transform3d;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>MockCamera</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(makeFinal = (true), level = AccessLevel.PRIVATE)
public class MockCamera extends Camera<PhotonCameraSim> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//

  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//

  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Limelight Camera Constructor.
   * @param Descriptor Real-world {@link #getDescriptor() descriptor} of the system, contains relevant constants to the operation of the module
   */
  public MockCamera(final Descriptor<PhotonCameraSim> Descriptor) {
    super(Descriptor);
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized void close() {

  }

  @Override
  public synchronized void update(final org.frc5411.lib.pattern.Report<@NonNull Transform3d> Record) {

  }
}