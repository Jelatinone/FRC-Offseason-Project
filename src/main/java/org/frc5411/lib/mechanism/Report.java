//------------------------------------------------------------------------[Package]------------------------------------------------------------------------//
package org.frc5411.lib.mechanism;
//-----------------------------------------------------------------------[Libraries]-----------------------------------------------------------------------//
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.Struct;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]-----------------------------------------------------------------------//
/**
 * 
 * <h1>Report</h1>
 * 
 * <p>Describes a general structure for a reported measurements of a {@link Component device} which produces measurement data of some type with 
 * a changing-value updated {@link Component#update() periodically}. Any types extending this class should be annotated with the relevant
 * {@link AutoLog annotation}.
 * @see LoggableInputs
 * @see Struct
 */
public abstract class Report<@NonNull Measurement extends WPISerializable> implements LoggableInputs, Cloneable {
  //------------------------------------------------------------------------[Fields]---------------------------------------------------------------------------//
  public volatile boolean Connected;

  public volatile double[] Timestamps;

  public @NonNull volatile Measurement Measurement;

  public @NonNull volatile Measurement[] Measurements;
  //------------------------------------------------------------------------[Methods]-------------------------------------------------------------------------//
  /**
   * Creates and returns a copy of this Report object, retaining all relevant information stored within, such as the
   * most-recent measurements, but is not the same specific instance.
   */
  public abstract Report<Measurement> clone();
}