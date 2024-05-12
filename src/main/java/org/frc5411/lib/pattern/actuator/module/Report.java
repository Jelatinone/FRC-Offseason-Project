//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.pattern.actuator.module;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import lombok.AccessLevel;
import lombok.Getter;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * <h1>Report</h1>
 * 
 * @see org.frc5411.lib.pattern.Report Report
 * 
 * @author Cody Washington
 */
@FieldDefaults(level = AccessLevel.PRIVATE)
@AutoLog
@Getter
public class Report extends org.frc5411.lib.pattern.actuator.Report<SwerveModuleState,SwerveModulePosition> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile double TranslationalVelocity;
  volatile double TranslationalVoltage;
  volatile double TranslationalAmperage;
  volatile boolean TranslationalConnected;

  volatile double RotationalVelocity;
  volatile double RotationalVoltage;
  volatile double RotationalAmperage;
  volatile boolean RotationalConnected;  
}
