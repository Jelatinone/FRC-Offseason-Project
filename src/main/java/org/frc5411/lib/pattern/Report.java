//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.pattern;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import lombok.experimental.FieldDefaults;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * 
 * <h1>Report</h1>
 * 
 * <p>Describes a general structure for a reported measurements of a {@link Component device} which produces measurement data of some type with 
 * a changing-value updated {@link Component#update(Report) periodically}. Any types extending this class should be annotated with the relevant
 * {@link AutoLog}, and {@link Getter} annotations.
 * 
 * @see LoggableInputs
 * @see Struct
 */
@FieldDefaults(level = AccessLevel.PRIVATE)
public abstract class Report<@NonNull Measured extends StructSerializable> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  @Getter volatile boolean Connected = (false);

  @Getter @NonNull volatile double[] Timestamps = {};

  @Getter @NonNull volatile Measured[] Measurements;

  /**
   * Shorthand for providing an empty instance of a report, with no relevant data stored inside.
   * @param <Measured> Type of the empty report, does not need to be specified in most cases
   * @return Empty report object
   */
  public static final <@NonNull Measured extends StructSerializable> Report<Measured> empty() {
    return new Report<Measured>() {};
  }

}
