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

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

import lombok.AccessLevel;
import lombok.Builder;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Descriptor</h1>
 * 
 * @see org.frc5411.lib.pattern.Descriptor Descriptor
 * 
 * @author Cody Washington
 */
@Builder(toBuilder = (true))
@FieldDefaults(level = AccessLevel.PROTECTED, makeFinal = (true))
public class Descriptor extends org.frc5411.lib.pattern.Descriptor<Component<Rotation3d>> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Vector<N3> Offset;
  Integer Identity;
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public Descriptor clone() {
    return new Descriptor(
      Offset,
      Identity
    );
  }
}
