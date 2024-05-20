//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.pattern;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.util.struct.StructSerializable;

import org.littletonrobotics.junction.AutoLog;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * 
 * <h1>Report</h1>
 * 
 * <p>Describes a general structure for a reported measurements of a {@link Component device} which produces measurement data of some type with 
 * a changing-value updated {@link Component#update(Report) periodically}. Any types extending this class should be annotated with the relevant
 * {@link AutoLog}, and {@link Getter} annotations. 
 * 
 * <p> Note that the contents of a given report, it's measurements, timestamps, and connection status are all values that can only be mutated
 * from Component of origin's access. <p>
 * 
 * @see StructSerializable
 */
@FieldDefaults(level = AccessLevel.PROTECTED)
@Getter
public abstract class Report<@NonNull Measured extends StructSerializable> {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  volatile boolean Connected = (false);

  @NonNull volatile double[] Timestamps = {};

  @NonNull volatile Measured[] Measurements;
  //------------------------------------------------------------------------[Methods]-------------------------------------------------------------------------//
  /**
   * Shorthand for providing an empty instance of a report, with no relevant data stored inside.
   * @param <Measured> Type of the empty report, does not need to be specified in most cases
   * @return Empty report object
   */
  public static final <@NonNull Measured extends StructSerializable> Report<Measured> empty() {
    return new Report<Measured>() {};
  }
}
