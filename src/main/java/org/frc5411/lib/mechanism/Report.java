//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * 
 * <h1>Report</h1>
 * 
 * <p>Describes a general structure for a reported measurements of a {@link Component device} which produces measurement data of some type with 
 * a changing-value updated {@link Component#update(Report) periodically}. Any types extending this class should be annotated with the relevant
 * {@link AutoLog annotation}.
 * @see LoggableInputs
 * @see Struct
 */
public class Report<@NonNull Measurement extends StructSerializable> implements LoggableInputs, Cloneable {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  public volatile boolean Connected;

  public volatile double[] Timestamps;

  public @NonNull volatile Measurement Measurement;

  public @NonNull volatile Measurement[] Measurements;
  //------------------------------------------------------------------------[Methods]-------------------------------------------------------------------------//
  /**
   * Creates and returns a copy of this Report object, retaining all relevant information stored within, such as the
   * most-recent measurements, but is not the same specific instance.
   * @return Copy of this object, but not the same instance
   */
  @Override
  public Report<Measurement> clone() {
    final var Copy = new Report<Measurement>();
    Copy.Connected = this.Connected;
    Copy.Timestamps = this.Timestamps;
    Copy.Measurement = this.Measurement;
    Copy.Measurements = this.Measurements;
    return Copy;
  }

  @Override
  public void toLog(final LogTable Table) {
    Table.put(("Connected"), Connected);
    Table.put(("Timestamps"), Timestamps);
    Table.put(("Measurement"), Measurement);
    Table.put(("Measurements"), Measurements);
  }
  @Override
  public void fromLog(final LogTable Table) {
    Connected = Table.get(("Connected")).getBoolean();
    Timestamps = Table.get(("Timestamps")).getDoubleArray();
    Measurement = Table.get(("Measurement"), Measurement);
    Measurements = Table.get(("Measurements"), Measurements);
  }
}