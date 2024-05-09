//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.observer.gyroscope;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation3d;

import org.littletonrobotics.junction.LogTable;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Report</h1>
 * c
 * <p>
 * 
 * @author Cody Washington
 */
public class Report extends org.frc5411.lib.mechanism.Report<Rotation3d> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  public volatile Rotation3d MeasurementVelocityRotationsMinute;
  public volatile Double AppliedVoltage = (0d);
  public volatile Double CurrentAmperage = (0d);
  public volatile Double TemperatureCelsius = (0d);
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//  
  @Override
  public void toLog(LogTable table) {
    
  }

  @Override
  public void fromLog(LogTable table) {
    
  }

  @Override
  public Report clone() {
    return (null);
  }
}
