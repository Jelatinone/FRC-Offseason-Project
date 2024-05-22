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
//--------------------------------------------------------------------------[Package]--------------------------------------------------------------------------//
package org.frc5411.lib.pattern;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import lombok.Builder;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
import lombok.experimental.FieldNameConstants;
import net.bytebuddy.utility.nullability.MaybeNull;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Descriptor</h1>
 * 
 * <p>As the name implies, Describes the mechanical constants of a given system that implements the {@link Component} type. This refers to values such
 * as gear reductions, ports, etc. The relevant constants are  contained within the object as a final property (without a value). The {@link Builder}
 * annotation should be applied (though entirely unforced by this type) so that these properties can easily be defined through by making a builder. Down-stream
 * implementations should, but are not explicitly required to make use of the {@link FieldDefaults} and {@link FieldNameConstants} annotations.
 * 
 * @see Cloneable
 * 
 * @author Cody Washington
 */
public abstract class Descriptor<@NonNull Described extends Component<?>> implements Cloneable {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Creates and returns a copy of this Report object, retaining all relevant information stored within, such as the
   * most-recent measurements, but is not the same specific instance.
   * @return Copy of this object, but not the same instance
   */
  public Descriptor<Described> clone() {
    return new Descriptor<Described>() {};
  }

  /**
   * Shorthand for providing an empty instance of a report, with no relevant data stored inside.
   * @param <Measured> Type of the empty report, does not need to be specified in most cases
   * @return Empty report object
   */
  public static final <@NonNull Described extends Component<?>> Descriptor<Described> empty() {
    return new Descriptor<Described>() {};
  }

  /**
   * Completes this object and turns it into a the Described type by passing it into the constructor of the specified Described type, returns null by default to
   * support constructors that require more arguments than a Descriptor or abstract types.
   * @return Instance of a Described type
   */
  public @MaybeNull Described complete() {
    return (null);
  }
  
}
