//----------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.robot2024;
//---------------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.frc5411.lib.schema.Singleton;
import org.frc5411.lib.schema.Subsystem;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
//--------------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 *
 *
 * <h1>Manager</h1>
 *
 * <p>Utility class handling the declaration and usage of subsystems at runtime.
 */
public final class Manager implements Singleton<Manager> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial
  private static final long serialVersionUID = 2389697764281159320L;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  private static volatile Manager Instance;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Manager Constructor.
   */
  private Manager() {

  } static {

  }
  //-----------------------------------------------------------------------[Methods]---------------------------------------------------------------------------//

  @Serial
  @Override
  public synchronized Manager readResolve() {
    return Instance;
  }
  
  @Serial
  @Override
  public synchronized void readObject(final ObjectInputStream Stream) throws IOException, ClassNotFoundException {
    Stream.defaultReadObject();
    Instance = (this);
  }

  @Override
  public synchronized void close() {
    synchronized(Manager.class) {
      Subsystem.getSubsystems().forEach((Subsystem) -> {
        try {
          Subsystem.close();
        } catch(final IOException Ignored) {}
      });      
      Instance = (null);
    }
  }

  @Override
  public Object clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getCanonicalName()));
  }
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized Manager getInstance() {
    Manager Result = Instance;
    if(Instance == (null)) {
      synchronized(Manager.class) {
        Result = Instance;
        if(Instance == (null)) {
          Instance = Result = new Manager();
        }
      }
    }
    return Result;
  }
}