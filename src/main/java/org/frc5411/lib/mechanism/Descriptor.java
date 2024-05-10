//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Descriptor</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public interface Descriptor<@NonNull Describes> extends Cloneable {

  /**
   * Creates and returns a copy of this Report object, retaining all relevant information stored within, such as the
   * most-recent measurements, but is not the same specific instance.
   * @return Copy of this object, but not the same instance
   */
  Descriptor<Describes> clone();

  /**
   * Completes this object and turns it into a the Described type by passing it into the constructor of the specified Described type, returns null by default to
   * support constructors that require more arguments than a Descriptor or abstract types.
   * @return Instance of a Described type
   */
  default Describes complete() {
    return (null);
  }
}
