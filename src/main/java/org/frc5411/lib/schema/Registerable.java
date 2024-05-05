//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.schema;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.NamedCommands;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Registerable</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public interface Registerable {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Registers a registerable named command, which has both a name and a command proxy retrieved through {@link Command#asProxy()}.
   */
  default void register() {
    NamedCommands.registerCommand(getName(), getCommand());
  }
  //---------------------------------------------------------------------[Accessors]-----------------------------------------------------------------------//
  /**
   * Proxy command of the named command to be registered as a named commands
   * @return Command to be registered
   */
  public Command getCommand();

  /**
   * Name of the command, must match the name used in PathPlanner for the command to be used.
   * @return Name (String) of the Named Command
   */
  public String getName();
  
}
