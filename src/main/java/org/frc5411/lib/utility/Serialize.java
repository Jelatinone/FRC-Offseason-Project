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
package org.frc5411.lib.utility;
//-----------------------------------------------------------------------[Libraries]-------------------------------------------------------------------------//

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Serialize</h1>
 * 
 * <p>Utility class for anything relating to serialization, houses a handful of static methods which perform simple, but nonetheless repetitive serialization related
 * functionality.
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class Serialize {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Converts a given object into it's equivalent serializable byte array format via {@link ByteArrayOutputStream}
   * @param Data Object to be converted into a byte array
   * @return Byte array converted from data
   */
  public static byte[] convert(final Object Data) {
    ByteArrayOutputStream Output = new ByteArrayOutputStream();
    try (ObjectOutputStream Input = new ObjectOutputStream(Output)) {
      Input.writeObject(Data);
      return Output.toByteArray();
    } catch (final IOException Exception) {
      Exception.printStackTrace();
    }
    throw new RuntimeException();
  }

  /**
   * Converts a given byte array into it's equivalent Object format via a via {@link ObjectInputStream}
   * @param Data Byte array to be converted to an object
   * @return Object converted from data
   */
  public static Object convert(final byte[] Data) {
    InputStream Input = new ByteArrayInputStream(Data);
    try (ObjectInputStream Object = new ObjectInputStream(Input)) {
      return Object.readObject();
    } catch (final IOException | ClassNotFoundException Exception) {
      Exception.printStackTrace();
    }
    throw new RuntimeException();
  }
}
