//------------------------------------------------------------------------[Package]----------------------------------------------------------------------------//
package org.frc5411.lib.schema;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import java.io.Closeable;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectStreamException;
import java.io.Serial;
import java.io.Serializable;

import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Singleton</h1>
 * 
 * <p>Represents a un-enforced singleton instance, which is {@link Serializable}, and {@link Closeable},
 * additionally, this class should define a <b>serialVersionUID</b> that differentiates this object from other 
 * objects which have been serialized into a {@link ObjectInputStream}.</p>
 * 
 * <p>Though not strictly enforced by this interface, a Singleton should define an accessor #getInstance() to 
 * statically retrieve the instance of a singleton type.
 * 
 * @author Cody Washington
 */
public interface Singleton<@NonNull Type> extends Serializable, Closeable, Cloneable {
  //----------------------------------------------------------------------[Methods]----------------------------------------------------------------------------//
  /**
   * Prevents potential serialization issues that may result in the creation of multiple un-intended instances of this class.
   * @return This instance, a singleton
   */
  @Serial
  Type readResolve() throws ObjectStreamException;

  /**
   * Prevents potential serialization issues that may result in the creation of multiple un-intended instances of this class.
   * @param Stream                  Part of the serialization mechanism which during deserialization provides a stream of raw bytes to be
   *                                converted into an object instance.
   * @throws IOException            When an exception has occurred during input or output
   * @throws ClassNotFoundException When the serialized object cannot be found
   */
  @Serial
  void readObject(ObjectInputStream Stream) throws IOException, ClassNotFoundException;
  
  /**
   * Closes this singleton instance, can be reopened by grabbing the instance again through an accessor.
   * @throws IOException When an exception has occurred during input or output
   */
  @Override
  void close() throws IOException;

  /**
   * Clones this singleton instance, throws an exception when this method is called because a singleton implicitly has only one instance
   * @return                            Nothing, an error is always thrown
   * @throws CloneNotSupportedException When the method is called, because a singleton may only permit a single instance
   */
  default Object clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getCanonicalName()));
  }
}
