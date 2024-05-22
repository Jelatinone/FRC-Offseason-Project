
//------------------------------------------------------------------------[Package]----------------------------------------------------------------------------//
package org.frc5411.lib.nouveau;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import java.io.Serial;
import java.nio.ByteBuffer;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.util.struct.Struct;
import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
import lombok.experimental.NonFinal;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>PhoenixRegister</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE, makeFinal = (true))
public class PhoenixRegister extends Thread implements Register {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial 
  static long serialVersionUID = 55742622883094958L;
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  @NonFinal static volatile PhoenixRegister Instance = (null);
  @NonFinal static volatile Serializable State;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Phoenix Register Constructor.
   */
  private PhoenixRegister() {
    State = new Serializable();
  }
  //-----------------------------------------------------------------------[Methods]---------------------------------------------------------------------------//
  @Override
  public synchronized void close() {
    synchronized(PhoenixRegister.class) {

    }
  }
  //-----------------------------------------------------------------------[Mutators]--------------------------------------------------------------------------//
  
  //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized PhoenixRegister getInstance() {
    PhoenixRegister Result = Instance;
    if(Instance == (null)) {
      synchronized(PhoenixRegister.class) {
        Result = Instance;
        if(Instance == (null)) {
          Instance = Result = new PhoenixRegister();
        }
      }
    }
    return Result;
  }
}
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
/**
 * <h1>Serializable</h1>
 * 
 * <p>
 * 
 */
final class Serializable extends Report {


}