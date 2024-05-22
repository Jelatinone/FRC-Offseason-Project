//------------------------------------------------------------------------[Package]----------------------------------------------------------------------------//
package org.frc5411.lib.nouveau;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import edu.wpi.first.util.struct.StructSerializable;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * 
 * <h1>Report</h1>
 * 
 * <p>
 * 
 * @see StructSerializable
 */
@FieldDefaults(level = AccessLevel.PROTECTED)
@Getter
public abstract class Report implements StructSerializable {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile int Samples = Integer.MIN_VALUE;
  volatile int Priority = Thread.MIN_PRIORITY;
  volatile double Period = Double.MIN_VALUE;  
  volatile double Timestamp = Double.MIN_VALUE;
  volatile boolean Running = Boolean.FALSE;
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Shorthand for providing an empty instance of a report, with no relevant data stored inside.
   * @return Empty report object
   */
  public static final Report empty() {
    return new Report() {};
  }
}
