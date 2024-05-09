//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.actuator.module;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import org.littletonrobotics.junction.LogTable;
import org.frc5411.lib.mechanism.Report;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Module</h1>
 * c
 * <p>
 * 
 * @author Cody Washington
 */
public class Record extends Report<SwerveModulePosition> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  public volatile Double TranslationalVelocityRotationsMinute = (0d);
  public volatile Double TranslationalAppliedVoltage = (0d);
  public volatile Double TranslationalCurrentAmperage = (0d);
  public volatile Double TranslationalTemperatureCelsius = (0d);
  public volatile Boolean TranslationalConnected = (false);

  public volatile Double RotationalVelocityRotationsMinute = (0d);
  public volatile Double RotationalAppliedVoltage = (0d);
  public volatile Double RotationalCurrentAmperage = (0d);
  public volatile Double RotationalTemperatureCelsius = (0d);
  public volatile Boolean RotationalConnected = (false);
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//  
  //TODO: Implement Unimplemented Methods
  @Override
  public void toLog(LogTable table) {
    
  }
  @Override
  public void fromLog(LogTable table) {
  }
  @Override
  public Report<@NonNull SwerveModulePosition> clone() {
    return null;
  }
}
