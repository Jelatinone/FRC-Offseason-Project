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
package org.frc5411.lib.nouveau;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import java.io.Serial;

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