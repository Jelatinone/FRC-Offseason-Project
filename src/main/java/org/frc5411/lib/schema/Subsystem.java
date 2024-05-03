//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.schema;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.locks.Lock;
import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.urcl.URCL;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Subsystem</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public abstract class Subsystem<Defined extends Registerable, State extends Enum<?>> extends SubsystemBase implements Singleton<Subsystem<Defined, State>> {
  //---------------------------------------------------------------------[Constants]-----------------------------------------------------------------------//
  private transient static final List<Subsystem<?,?>> SUBSYSTEMS = new ArrayList<>();
  private final Lock OPERATION_LOCK;
  //-------------------------------------------------------------------[Constructor(s)]--------------------------------------------------------------------//
  /**
   * Subsystem Constructor.
   * @param Lock Lock which ensures a blocking operation during {@link #periodic()} if a previous call has not yet ended.
   */
  protected Subsystem(final Lock Lock, final String Name) {
    super(Objects.requireNonNull(Name));
    OPERATION_LOCK = Objects.requireNonNull(Lock);
    SUBSYSTEMS.add(this);
  }
  //----------------------------------------------------------------------[Abstract]------------------------------------------------------------------------//
  /**
   * Updates relevant {@link Logger loggable} values using {@link Logger#recordOutput(String, Type)} that may have changed during runtime. This
   * is inclusive of values such as encoder values, motor outputs, etc, that are not automatically logged (such as {@link URCL}) that may be useful during
   * the debugging process.
   */
  public abstract void update();

  /**
   * Provides the list of enum values containing all of the named commands registered under this subsystem instance
   * @return Enum of Named Commands
   */
  public abstract List<Defined> getCommands();

  /**
   * Provides the enum of the current state of this subsystem instance.
   * @return State of this instance
   */
  public abstract State getState();
  //---------------------------------------------------------------------[Accessors]-----------------------------------------------------------------------//
  /**
   * Provides the lock member-variable of this subsystem used during it's periodic operations.
   * @return Synchronization lock of this subsystem
   */
  public final Lock getLock() {
    return OPERATION_LOCK;
  }

  @Override
  public final Object clone() throws CloneNotSupportedException {
    return super.clone();
  }

  /**
   * Provides a list (ordered) of all constructed subsystems.
   * @return List of subsystems
   */
  public static final List<Subsystem<?,?>> getSubsystems() {
    return SUBSYSTEMS;
  }
}
