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
import java.util.function.Function;

import lombok.Builder;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
import lombok.experimental.FieldNameConstants;
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
   * Creates and returns a copy of this Descriptor object, retaining all relevant information stored within, such as the
   * relevant constants, but is not the same specific instance.
   * @return Copy of this object, but not the same instance
   */
  public Descriptor<Described> clone() {
    return new Descriptor<>() {};
  }

  /**
   * Shorthand for providing an empty instance of a report, with no relevant data stored inside.
   * @param <Described> Type of the empty report, does not need to be specified in most cases
   * @return Empty report object
   */
  public static <@NonNull Described extends Component<?>> Descriptor<Described> empty() {
    return new Descriptor<>() {};
  }

  /**
   * Completes this descriptor object by transforming it into the described type by passing it into the generator (constructor) of the specified downstream type; this
   * behavior is only supported for {@link Component implementations} which reference only a constructor of this descriptor type within their constructor.
   * @param <Downstream> Type which extends the descriptor, filling out all of the relevant fields for the product at it's implementation level
   * @param <Produces>   Type which extends the described type, resolves issues with components being created rather than the desired type via an additional cast
   * @param Generator    Functional type generator which accepts a {@code Descriptor} argument, and produces the {@code Described} type to be desired.
   * @return Instance of a Described type
   */
  @SuppressWarnings("unchecked")
  public @NonNull <Downstream extends Descriptor<Described>, Produces extends Described> Produces complete(final Function<? super Downstream,? extends Produces> Generator) {
    return (Produces) Generator.apply((Downstream) this);
  }
}
