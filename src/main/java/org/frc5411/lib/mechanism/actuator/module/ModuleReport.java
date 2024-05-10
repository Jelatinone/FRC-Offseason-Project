//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism.actuator.module;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.math.kinematics.SwerveModulePosition;

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
public class ModuleReport extends Report<SwerveModulePosition> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile Double TranslationalVelocityRotationsMinute = (0d);
  volatile Double TranslationalAppliedVoltage = (0d);
  volatile Double TranslationalCurrentAmperage = (0d);
  volatile Double TranslationalTemperatureCelsius = (0d);
  volatile Boolean TranslationalConnected = (false);

  volatile Double RotationalVelocityRotationsMinute = (0d);
  volatile Double RotationalAppliedVoltage = (0d);
  volatile Double RotationalCurrentAmperage = (0d);
  volatile Double RotationalTemperatureCelsius = (0d);
  volatile Boolean RotationalConnected = (false);
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//  
  @Override
  public void toLog(final LogTable Table) {
    super.toLog(Table);
    Table.put(("Translational-Velocity-Rotations-Minute"), TranslationalVelocityRotationsMinute);
    Table.put(("Translational-Applied-Voltage"), TranslationalAppliedVoltage);
    Table.put(("Translational-Current-Amperage"), TranslationalCurrentAmperage);
    Table.put(("Translational-Temperature-Celsius"), TranslationalTemperatureCelsius);
    Table.put(("Translational-Connected"), TranslationalConnected);

    Table.put(("Rotational-Velocity-Rotations-Minute"), RotationalVelocityRotationsMinute);
    Table.put(("Rotational-Applied-Voltage"), RotationalAppliedVoltage);
    Table.put(("Rotational-Current-Amperage"), RotationalCurrentAmperage);
    Table.put(("Rotational-Temperature-Celsius"), RotationalTemperatureCelsius);
    Table.put(("Rotational-Connected"), RotationalConnected);
  }

  @Override
  public void fromLog(final LogTable Table) {
    super.fromLog(Table);
    TranslationalVelocityRotationsMinute = Table.get(("Translational-Velocity-Rotations-Minute")).getDouble();
    TranslationalAppliedVoltage = Table.get(("Translational-Applied-Voltage")).getDouble();
    TranslationalCurrentAmperage = Table.get(("Translational-Current-Amperage")).getDouble();
    TranslationalTemperatureCelsius = Table.get(("Translational-Temperature-Celsius")).getDouble();
    TranslationalConnected = Table.get(("Translational-Connected")).getBoolean();

    RotationalVelocityRotationsMinute = Table.get(("Rotational-Velocity-Rotations-Minute")).getDouble();
    RotationalAppliedVoltage = Table.get(("Rotational-Applied-Voltage")).getDouble();
    RotationalCurrentAmperage = Table.get(("Rotational-Current-Amperage")).getDouble();
    RotationalTemperatureCelsius = Table.get(("Rotational-Temperature-Celsius")).getDouble();
    RotationalConnected = Table.get(("Rotational-Connected")).getBoolean();
  }

  @Override
  public ModuleReport clone() {
    return (null);
  }
}
