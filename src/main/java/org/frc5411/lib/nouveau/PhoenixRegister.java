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
package org.frc5411.lib.nouveau;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.util.struct.Struct;

import com.ctre.phoenix6.StatusSignal;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serial;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import lombok.AccessLevel;
import lombok.NonNull;
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
public class PhoenixRegister extends Thread implements Register<StatusSignal<?>,Serializable> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @Serial 
  static long serialVersionUID = 55742622883094958L;
  List<StatusSignal<?>> SIGNALS;
  List<DoubleCircularBuffer> TIMESTAMPS;  
  List<DoubleCircularBuffer> BUFFERS;
  ReadWriteLock BUFFER_LOCK;
  ReadWriteLock SIGNAL_LOCK;  
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  @NonFinal static volatile PhoenixRegister Instance = (null);
  @NonFinal static volatile Serializable State;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Phoenix Register Constructor.
   */
  private PhoenixRegister() {
    State = new Serializable();
    TIMESTAMPS = new ArrayList<>();
    BUFFERS = new ArrayList<>();
    SIGNALS = new ArrayList<>();
    BUFFER_LOCK = new ReentrantReadWriteLock((true));
    SIGNAL_LOCK = new ReentrantReadWriteLock((true));  
    setDaemon((true));
    setName(getClass().getCanonicalName());
  } static {
    
  }
  //-----------------------------------------------------------------------[Methods]---------------------------------------------------------------------------//
  @Serial
  @Override
  public synchronized PhoenixRegister readResolve() {
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
    halt();
    synchronized(PhoenixRegister.class) {
      TIMESTAMPS.forEach(DoubleCircularBuffer::clear);
      BUFFERS.forEach(DoubleCircularBuffer::clear);
      SIGNALS.clear();
      Instance = (null);
    }
  }

  @Override
  public final Object clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getCanonicalName()));
  }

  @Override
  public synchronized void halt(final Long Timeout) {
    try {
      join(Timeout);
    } catch(final InterruptedException Exception) {
      currentThread().interrupt();
    }
  }

  @Override
  public DoubleCircularBuffer register(@NonNull StatusSignal<?> Signal) {
    return null;
  }

  @Override
  public DoubleCircularBuffer timestamp() {
    return null;
  }

  @Override
  public synchronized void run() {
    synchronized(Instance) {
      while(isAlive() && !isInterrupted()) {
        synchronized(PhoenixRegister.class) {

        }
      }
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

  @Override
  public ReadWriteLock getBufferLock() {
    return BUFFER_LOCK;
  }

  @Override
  public ReadWriteLock getSignalLock() {
    return SIGNAL_LOCK;
  }

  @Override
  public Serializable getReport() {
    return State;
  }
}
//-----------------------------------------------------------------------[External]----------------------------------------------------------------------------//
/**
 * <h1>Serializable</h1>
 * 
 * <p>Struct serializable instance of a report
 * 
 * @see SerializableStruct
 * 
 */
final class Serializable extends Report {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  public static final SerializableStruct STRUCT = new SerializableStruct(); 
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  protected volatile int Failed = Integer.MIN_VALUE;
  //-----------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   * <h1>SerializableStruct</h1>
   * 
   * <p>Describes a struct serializable instance of a {@link Serializable} instance, which can be sent over the network as a struct.
   */
  static final class SerializableStruct implements Struct<Serializable> {
    //-----------------------------------------------------------------------[Methods]---------------------------------------------------------------------------//
    @Override
    public Serializable unpack(final ByteBuffer Buffer) {
      final var State = new Serializable();
      State.Failed = Buffer.getInt();
      State.Samples = Buffer.getInt();
      State.Priority = Buffer.getInt();
      State.Period = Buffer.getDouble();
      State.Timestamp = Buffer.getDouble();
      State.Running = Buffer.get() != (0);
      return State;
    }

    @Override
    public void pack(final ByteBuffer Buffer, final Serializable Value) {
      Buffer.putInt(Value.Failed);
      Buffer.putInt(Value.Samples);
      Buffer.putInt(Value.Priority);
      Buffer.putDouble(Value.Period);
      Buffer.putDouble(Value.Timestamp);
      Buffer.putDouble((byte) (Value.Running? (1): (0)));
    }
    //-----------------------------------------------------------------------[Accessors]-------------------------------------------------------------------------//
    @Override
    public Class<Serializable> getTypeClass() {
      return Serializable.class;
    }

    @Override
    public String getTypeString() {
      return "STRUCT:PhoenixRegister.Serializable"; // TODO
    }

    @Override
    public int getSize() {
      return kSizeBool + kSizeInt32 * (3) + kSizeDouble * (2);
    }

    @Override
    public String getSchema() {
      return "int32 Failed;int32 Samples;int32 Priority;double Period;double Timestamp;bool Running";
    }
  }
}