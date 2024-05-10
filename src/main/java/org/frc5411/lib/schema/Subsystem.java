//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.schema;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.jcabi.aspects.Async;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.urcl.URCL;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.locks.Lock;

import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Subsystem</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public abstract class Subsystem<@NonNull Defined extends Registrable, @NonNull State extends Enum<?>> extends SubsystemBase implements Singleton<Subsystem<Defined, State>> {
  //---------------------------------------------------------------------[Constants]-----------------------------------------------------------------------//
  private static final List<Subsystem<?,?>> SUBSYSTEMS = new ArrayList<>();
  private final Lock OPERATION_LOCK;
  //-------------------------------------------------------------------[Constructor(s)]--------------------------------------------------------------------//
  /**
   * Subsystem Constructor.
   * @param Lock Lock which ensures a blocking operation during {@link #periodic()} if a previous call has not yet ended.
   * @param Name Referenceable name by which to refer the subsystem, this is an entirely objective value to programmer preferences
   */
  protected Subsystem(final Lock Lock, final String Name) {
    super(Objects.requireNonNull(Name));
    OPERATION_LOCK = Objects.requireNonNull(Lock);
    SUBSYSTEMS.add(this);
  }

  /**
   * Subsystem Constructor.
   * @param Lock Lock which ensures a blocking operation during {@link #periodic()} if a previous call has not yet ended.
   */
  protected Subsystem(final Lock Lock) {
    super();
    OPERATION_LOCK = Objects.requireNonNull(Lock);
    SUBSYSTEMS.add(this);
  }
  //---------------------------------------------------------------------[Mutators]------------------------------------------------------------------------//
  /**
   * Updates relevant {@link Logger loggable} values using {@link Logger#recordOutput(String, edu.wpi.first.util.WPISerializable)} that may have changed during runtime. This
   * is inclusive of values such as encoder values, motor outputs, etc., that are not automatically logged (such as {@link URCL}) that may be useful during
   * the debugging process.
   */
  @Async
  public abstract void update();

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
  //---------------------------------------------------------------------[Accessors]-----------------------------------------------------------------------//
  /**
   * Provides the lock member-variable of this subsystem used during it's {@link #periodic() periodic} operations.
   * @return Synchronization lock of this subsystem
   */
  public final Lock getLock() {
    return OPERATION_LOCK;
  }

  @Override
  public final Object clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getCanonicalName()));
  }

  /**
   * Provides a list (ordered) of all constructed subsystems.
   * @return List of subsystems
   */
  public static List<Subsystem<?,?>> getSubsystems() {
    return SUBSYSTEMS;
  }
}
