//------------------------------------------------------------------------[License]----------------------------------------------------------------------------//
// Copyright 2024 Cody Washington
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//------------------------------------------------------------------------[Package]----------------------------------------------------------------------------//
package org.frc5411.lib.instrument.gyroscope.archetype;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.instrument.gyroscope.Descriptor;
import org.frc5411.lib.instrument.gyroscope.Gyroscope;
import org.frc5411.lib.instrument.gyroscope.Report;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.nouveau.StandardRegister;
import org.frc5411.lib.utility.Figures;

import edu.wpi.first.math.geometry.Rotation3d;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import java.util.Optional;
import java.util.Queue;
import java.util.stream.IntStream;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Pigeon Gyroscope</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(makeFinal = (true), level = AccessLevel.PRIVATE)
public class PigeonGyroscope extends Gyroscope<Pigeon2> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  Queue<Optional<Number>> YAW_POSITIONS, PITCH_POSITIONS, ROLL_POSITIONS;
  Queue<Double> UPDATE_TIMESTAMPS;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Simulated Module Constructor.
   * @param Descriptor Real-world {@link #getDescriptor() descriptor} of the system, contains relevant constants to the operation of the module
   */
  public PigeonGyroscope(final Descriptor<Pigeon2> Descriptor) {
    super(Descriptor);
    YAW_POSITIONS = StandardRegister
      .getInstance()
      .register(() -> Optional.of(
        Descriptor.Hardware.getYaw().getValue()
      ));
    PITCH_POSITIONS = StandardRegister
      .getInstance()
      .register(() -> Optional.of(
        Descriptor.Hardware.getPitch().getValue()
      ));
    ROLL_POSITIONS = StandardRegister
      .getInstance()
      .register(() -> Optional.of(
        Descriptor.Hardware.getRoll().getValue()
      ));
    UPDATE_TIMESTAMPS = StandardRegister
      .getInstance()
      .timestamp();
    configure();
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized void configure() {
    synchronized(this) {
      getDescriptor().Hardware
        .getYaw()
        .setUpdateFrequency(StandardRegister.getInstance().getFrequency() / (20));
      getDescriptor().Hardware
        .getPitch()
        .setUpdateFrequency(StandardRegister.getInstance().getFrequency() / (20));
      getDescriptor().Hardware
        .getRoll()
        .setUpdateFrequency(StandardRegister.getInstance().getFrequency() / (20));
      getDescriptor().Hardware
        .optimizeBusUtilization();

      getDescriptor().Hardware.getConfigurator().apply(new Pigeon2Configuration());
    }
  }

  @Override
  public synchronized void reset() {
    getDescriptor().Hardware.reset();
  }

  @Override
  public synchronized void close() {
    getDescriptor().Hardware.close();

    YAW_POSITIONS.clear();
    PITCH_POSITIONS.clear();
    ROLL_POSITIONS.clear();
    UPDATE_TIMESTAMPS.clear();
  }

  @Override
  public synchronized void update(final org.frc5411.lib.pattern.Report<@NonNull Rotation3d> Record) {
    final var Article = (Report) Record;

    synchronized(Article) {
      Article.setConnected(!getDescriptor().Hardware.getFault_Hardware().getValue());
      Article.setVelocity(new Rotation3d(
        getDescriptor().Hardware.getAngularVelocityXDevice().getValue(),
        getDescriptor().Hardware.getAngularVelocityYDevice().getValue(), 
        getDescriptor().Hardware.getAngularVelocityZDevice().getValue()));
  
      final double[] Yaw, Pitch, Roll;
      synchronized(YAW_POSITIONS) {
        Yaw = YAW_POSITIONS
          .stream()
          .mapToDouble((Position) -> 
            Position.orElse(Double.NaN).doubleValue() - getDescriptor().Offset.getZ())
          .toArray();
        YAW_POSITIONS.clear();
      }
      synchronized(PITCH_POSITIONS) {
        Pitch = PITCH_POSITIONS
          .stream()
          .mapToDouble((Position) -> 
            Position.orElse(Double.NaN).doubleValue() - getDescriptor().Offset.getY())
          .toArray();
        PITCH_POSITIONS.clear();
      }
      synchronized(ROLL_POSITIONS) {
        Roll = ROLL_POSITIONS
          .stream()
          .mapToDouble((Position) -> 
            Position.orElse(Double.NaN).doubleValue() - getDescriptor().Offset.getX())
          .toArray();
        ROLL_POSITIONS.clear();
      }
      synchronized(UPDATE_TIMESTAMPS) {
        Article.setTimestamps(UPDATE_TIMESTAMPS
          .stream()
          .mapToDouble(Double::doubleValue)
          .toArray());
        UPDATE_TIMESTAMPS.clear();
      }        
      Article.setMeasurements(IntStream.range((0), (int) Figures.minimum(Yaw.length, Pitch.length, Roll.length)).mapToObj((Index) -> 
        new Rotation3d(
          Roll[Index],
          Pitch[Index],
          Yaw[Index])
      ).toArray(Rotation3d[]::new));
    }
  }
}
