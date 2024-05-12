//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.pattern.actuator;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.util.struct.StructSerializable;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.NonNull;
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
@Getter
public abstract class Report<@NonNull Reference extends StructSerializable, @NonNull Measurement extends StructSerializable> extends org.frc5411.lib.pattern.Report<Measurement> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  @NonNull volatile Reference Effort;
  
  @NonNull volatile Reference Demand; 
}
