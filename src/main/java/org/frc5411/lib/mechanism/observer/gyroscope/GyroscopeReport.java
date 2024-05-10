//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.observer.gyroscope;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation3d;

import org.frc5411.lib.mechanism.Report;
import org.littletonrobotics.junction.LogTable;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Report</h1>
 * c
 * <p>
 * 
 * @author Cody Washington
 */
public class GyroscopeReport extends Report<Rotation3d> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile Rotation3d MeasurementVelocityRotationsMinute;
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//  
  @Override
  public void toLog(final LogTable Table) {
    super.toLog(Table);
    Table.put(("VelocityRotationsMinute"), MeasurementVelocityRotationsMinute);
  }

  @Override
  public void fromLog(final LogTable Table) {
    super.fromLog(Table);
    MeasurementVelocityRotationsMinute = Table.get(("VelocityRotationsMinute"), MeasurementVelocityRotationsMinute);
  }

  @Override
  public GyroscopeReport clone() {
    final var Copy = (GyroscopeReport) super.clone();
    Copy.MeasurementVelocityRotationsMinute = this.MeasurementVelocityRotationsMinute;
    return Copy;
  }
}
