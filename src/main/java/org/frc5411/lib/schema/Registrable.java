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
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.NamedCommands;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Registrable</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
public interface Registrable {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Registers a registrable named command, which has both a name and a command proxy retrieved through {@link Command#asProxy()}.
   */
  default void register() {
    NamedCommands.registerCommand(getName(), getCommand());
  }
  //---------------------------------------------------------------------[Accessors]---------------------------------------------------------------------------//
  /**
   * Proxy command of the named command to be registered as a named commands
   * @return Command to be registered
   */
  Command getCommand();

  /**
   * Name of the command, must match the name used in PathPlanner for the command to be used.
   * @return Name (String) of the Named Command
   */
  String getName();
}
