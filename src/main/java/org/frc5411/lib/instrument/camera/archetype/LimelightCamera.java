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
package org.frc5411.lib.instrument.camera.archetype;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.frc5411.lib.instrument.camera.Camera;
import org.frc5411.lib.instrument.camera.Descriptor;
import org.frc5411.lib.instrument.camera.Report;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableValue;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.Objects;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>LimelightCamera</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(makeFinal = (true), level = AccessLevel.PRIVATE)
public class LimelightCamera extends Camera<NetworkTable> {
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//

  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Limelight Camera Constructor.
   * @param Descriptor Real-world {@link #getDescriptor() descriptor} of the system, contains relevant constants to the operation of the module
   */
  public LimelightCamera(final Descriptor<NetworkTable> Descriptor) {
    super(Descriptor);

  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized void configure() {

  }

  @Override
  public synchronized void close() {

  }

  /**
   * Requests a {@link NetworkTableValue network table value} from this Camera's {@link Descriptor descriptor} given the string name of the entry. 
   * @param Property Name of the property in the table as an {@link Accessible enum} value that can be converted to string form to access via {@link NetworkTable#getEntry(String)}
   * @return Optional value of the requested property value, if found
   */
  public synchronized Optional<NetworkTableValue> access(final Accessible Property) {
    return Optional
      .ofNullable(getDescriptor().Hardware.getEntry(Property.get()).getValue());
  }

  /**
   * Mutates a {@link NetworkTableValue network table value} from this Camera's {@link Descriptor descriptor} given the string name of the entry. 
   * @param <Type>   Type of the value to mutate, the supplied value must be an instance of this type
   * @param Property Name of the property in the table as an {@link Mutable enum} value that can be converted to string form to access via {@link NetworkTable#getEntry(String)}
   * @param Value    Value, of the expected type, to set the mutable entry to
   */
  public synchronized <Type> void mutate(final Mutable Property, final Type Value) {
    getDescriptor().Hardware
      .getEntry(Property.get()).setValue(Value);
  }

  @Override
  public synchronized void update(final org.frc5411.lib.pattern.Report<@NonNull Transform2d> Record) {
    final var Article = (Report) Record;

    synchronized(Article) {
      
    }
  }
  //-----------------------------------------------------------------------[Mutators]--------------------------------------------------------------------------//

  //----------------------------------------------------------------------[Internal]---------------------------------------------------------------------------//
  /**
   * <h1>MutatorKey</h1>
   */
  public enum Mutable implements Supplier<String> {
    //---------------------------------------------------------------------[Values]----------------------------------------------------------------------------//
    PRIMARY_APRIL_TAG_ID(("priorityid")),
    LED_MODE(("ledMode")),
    CAMERA_MODE(("camMode")),
    PIPELINE_MODE(("pipeline")),
    STREAMING_MODE(("stream")),
    TAKE_SNAPSHOT(("snapshot")),
    IMAGE_CROP(("crop")),
    ROBOT_POSE(("camerapose_robotspace_set")),
    ROBOT_ORIENTATION_UNITS(("robot_orientation_set")),
    FICIDUAL_ID_FILTERS(("fiducial_id_filters_set"));
    //--------------------------------------------------------------------[Constants]--------------------------------------------------------------------------//
    private final String API_KEY_NAME;
    //------------------------------------------------------------------[Constructor(s)]-----------------------------------------------------------------------//
    /**
     * Mutable Constructor
     * @param Key Name of Key, valid key to pull limelight data from.
     */
    Mutable(final String Key) {
      API_KEY_NAME = Objects
        .requireNonNull(Key);
    }
    //--------------------------------------------------------------------[Accessors]--------------------------------------------------------------------------//
    /**
     * Provides the underlying API key of a given key object.
     * @return Key as a string representation
     */
    public String get() {
      return API_KEY_NAME;
    }
  }
  /**
   * <h1>AccessorKey</h1>
   */
  public enum Accessible implements Supplier<String> {
    //---------------------------------------------------------------------[Values]----------------------------------------------------------------------------//
    HAS_TARGETS(("tv")),
    HORIZONTAL_TARGET_OFFSET_CROSSHAIR(("tx")),
    VERTICAL_TARGET_OFFSET_CROSSHAIR(("ty")),
    HORIZONTAL_TARGET_OFFSET_PRINCIPAL(("txnc")),
    VERTICAL_TARGET_OFFSET_PRINCIPAL(("tync")),
    TARGET_AREA(("ta")),
    CURRENT_PIPELINE_LATENCY(("tl")),
    CAPTURE_PIPELINE_LATENCY(("cl")),
    TARGET_BOUND_SHORTEST_LENGTH(("tshort")),
    TARGET_BOUND_LONGEST_LENGTH(("tlong")),
    TARGET_BOUND_HORIZONTAL_LENGTH(("thor")),
    TARGET_BOUND_VERTICAL_LENGTH(("tvert")),
    PIPELINE_INDEX(("getpipe")),
    TARGETING_DATA(("json")),
    NEURAL_DETECTOR_CLASS_RESULT(("tclass")),
    AVERAGE_HSV_VALUE_CROSSHAIR(("tc")),
    HEART_BEAT_VALUE(("hb")),
    HARDWARE_METRICS(("hw")),
    ROBOT_POSE_FIELD_RELATIVE(("botpose")),
    ROBOT_POSE_FIELD_RELATIVE_BLUE_ORIGIN(("botpose_wpiblue")),
    ROBOT_POSE_FIELD_RELATIVE_RED_ORIGIN(("botpose_wpired")),
    ROBOT_POSE_FIELD_RELATIVE_ORB(("botpose_orb")),
    ROBOT_POSE_FIELD_RELATIVE_ORB_BLUE_ORIGIN(("botpose_orb_wpiblue")),
    ROBOT_POSE_FIELD_RELATIVE_ORB_RED_ORIGIN(("botpose_orb_wpired")),
    CAMERA_POSE_TARGET_RELATIVE(("targetpose_cameraspace")),
    TARGET_POSE_CAMERA_RELATIVE(("targetpose_cameraspace")),
    TARGET_POSE_ROBOT_RELATIVE(("targetpose_robotspace")),
    ROBOT_POSE_PRIMARY_VIEW_RELATIVE(("botpose_targetspace")),
    CAMERA_TRANSFORM(("camerapose_robotspace")),
    PRIMARY_VIEW_TAG(("tid")),
    TARGET_CORNERS(("tcornxy")),
    RAW_TARGETS(("rawtargets")),
    RAW_FICIDUAL(("rawfiducials")),
    RAW_CROSSHAIR_A_HORIZONTAL(("cx0")),
    RAW_CROSSHAIR_A_VERTICAL(("cy0")),
    RAW_CROSSHAIR_B_HORIZONTAL(("cx1")),
    RAW_CROSSHAIR_B_VERTICAL(("cy1"));
    //--------------------------------------------------------------------[Constants]--------------------------------------------------------------------------//
    private final String API_KEY_NAME;
    //------------------------------------------------------------------[Constructor(s)]-----------------------------------------------------------------------//
    /**
     * Accessible Constructor
     * @param Key Name of Key, valid key to pull limelight data from.
     */
    Accessible(final String Key) {
      API_KEY_NAME = Objects
        .requireNonNull(Key);
    }
    //--------------------------------------------------------------------[Accessors]--------------------------------------------------------------------------//
    /**
     * Provides the underlying API key of a given key object.
     * @return Key as a string representation
     */
    public String get() {
      return API_KEY_NAME;
    }
  }
}