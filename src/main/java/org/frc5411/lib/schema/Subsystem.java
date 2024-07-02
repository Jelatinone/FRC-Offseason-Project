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
package org.frc5411.lib.schema;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.jcabi.aspects.Async;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.urcl.URCL;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.locks.ReadWriteLock;

import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Subsystem</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public abstract class Subsystem<@NonNull Defined extends Registrable, @NonNull State extends Enum<?>> extends SubsystemBase implements Singleton<Subsystem<Defined, State>> {
  //---------------------------------------------------------------------[Constants]---------------------------------------------------------------------------//
  private static final List<Subsystem<?,?>> SUBSYSTEMS = new ArrayList<>();
  private final ReadWriteLock OPERATION_LOCK;
  //-------------------------------------------------------------------[Constructor(s)]------------------------------------------------------------------------//
  /**
   * Subsystem Constructor.
   * @param Lock Lock which ensures a blocking operation during {@link #periodic()} if a previous call has not yet ended.
   * @param Name Referencable name by which to refer the subsystem, this is an entirely objective value to programmer preferences
   */
  protected Subsystem(final ReadWriteLock Lock, final String Name) {
    super(Objects.requireNonNull(Name.strip()));
    OPERATION_LOCK = Objects.requireNonNull(Lock);
    SUBSYSTEMS.add(this);
  }

  /**
   * Subsystem Constructor.
   * @param Lock Lock which ensures a blocking operation during {@link #periodic()} if a previous call has not yet ended.
   */
  protected Subsystem(final ReadWriteLock Lock) {
    super();
    OPERATION_LOCK = Objects.requireNonNull(Lock);
    SUBSYSTEMS.add(this);
  }
  //----------------------------------------------------------------------[Methods]----------------------------------------------------------------------------//
  /**
   * Updates relevant {@link Logger loggable} values using {@link Logger#recordOutput(String, edu.wpi.first.util.WPISerializable)} that may have changed during runtime. This
   * is inclusive of values such as encoder values, motor outputs, etc., that are not automatically logged (such as {@link URCL}) that may be useful during
   * the debugging process.
   */
  @Async
  public abstract void update();
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Provides the list of enum values containing all the named commands registered under this subsystem instance
   * @return Enum of Named Commands
   */
  public abstract List<Defined> getCommands();

  /**
   * Provides the enum of the current state of this subsystem instance.
   * @return State of this instance
   */
  public abstract State getState();
  
  /**
   * Provides the lock member-variable of this subsystem used during it's {@link #periodic() periodic} operations.
   * @return Synchronization lock of this subsystem
   */
  public final ReadWriteLock getLock() {
    return OPERATION_LOCK;
  }

  @Override
  public final Subsystem<Defined,State> clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getSigners()));
  }

  /**
   * Provides a list (ordered) of all constructed subsystems.
   * @return List of subsystems
   */
  public static List<Subsystem<?,?>> getSubsystems() {
    return SUBSYSTEMS;
  }
}
