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
import org.frc5411.lib.nouveau.IdentityRegister;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Queue;
import java.util.function.Supplier;

import lombok.AccessLevel;
import lombok.NonNull;
import lombok.experimental.FieldDefaults;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>MockCamera</h1>
 * 
 * <p>
 * 
 * @author Cody Washington
 */
@FieldDefaults(makeFinal = (true), level = AccessLevel.PRIVATE)
public class MockCamera extends Camera<Supplier<Pose2d>> {
  //-----------------------------------------------------------------------[Registers]-------------------------------------------------------------------------//
  static IdentityRegister<PhotonPipelineResult> RESULT_REGISTER;
  //-----------------------------------------------------------------------[Constants]-------------------------------------------------------------------------//
  VisionSystemSim WORLD;
  PhotonPoseEstimator ESTIMATOR;
  PhotonCameraSim SIMULATOR;

  Queue<PhotonPipelineResult> CAMERA_RESULTS;
  //---------------------------------------------------------------------[Constructor(s)]----------------------------------------------------------------------//
  /**
   * Mock Camera Constructor.
   * @param Descriptor Real-world {@link #getDescriptor() descriptor} of the system, contains relevant constants to the operation of the camera
   */
  public MockCamera(final Descriptor<Supplier<Pose2d>> Descriptor) {
    super(Descriptor);

    final var Layout = AprilTagFieldLayout
      .loadField(AprilTagFields.kDefaultField);
    final var Properties = new SimCameraProperties();
    Properties.setCalibration(
      (960), 
      (720), 
      Rotation2d.fromDegrees((75)));
    Properties.setCalibError(
      Math.random(),
      Math.random());
    Properties.setFPS((100));
    Properties.setAvgLatencyMs((90));
    Properties.setLatencyStdDevMs((15));

    SIMULATOR = new PhotonCameraSim(new PhotonCamera(getIdentity()), Properties);
    SIMULATOR.enableDrawWireframe((true));

    ESTIMATOR = new PhotonPoseEstimator(
      Layout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      SIMULATOR
        .getCamera(),
      getDescriptor().Position);
    ESTIMATOR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    WORLD = new VisionSystemSim(getIdentity());
    WORLD.addAprilTags(Layout);
    WORLD.addCamera(SIMULATOR, getDescriptor().Position);

    CAMERA_RESULTS = RESULT_REGISTER
      .register(() -> {
          WORLD.update(new Pose3d()); // <--- Call causing wait!
          return SIMULATOR
            .getCamera()
            .getLatestResult();
        }
      );
  } static {
    RESULT_REGISTER = new IdentityRegister<>();
  }
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  @Override
  public synchronized void close() {
    SIMULATOR.close();
    CAMERA_RESULTS.clear(); 
  }

  @Override
  public synchronized void update(final org.frc5411.lib.pattern.Report<@NonNull Transform3d> Record) {
    final var Article = (Report) Record;

    synchronized(Article) {
      Article.setConnected(SIMULATOR.getCamera().isConnected());
      Article.setPipeline(SIMULATOR.getCamera().getPipelineIndex());

      synchronized(CAMERA_RESULTS) {
        Article.setLatency(
          CAMERA_RESULTS
            .stream()
            .reduce((Current, Next) -> Next)
            .map((Result) -> Result.getLatencyMillis() / 1E3D)
            .orElse((-1D))
        );
        Article.setObservations(
          CAMERA_RESULTS
            .stream()
            .map((Measurement) -> 
              ESTIMATOR
                .update(Measurement)
                .map((Position) -> Position.estimatedPose)
                .orElse(new Pose3d()))
            .toArray(Pose3d[]::new)
        );
        Article.setMeasurements(
          CAMERA_RESULTS
            .stream()
            .map((Measurement) -> 
              Measurement
                .getBestTarget()
                .getBestCameraToTarget())
            .toArray(Transform3d[]::new)
        );
        Article.setTimestamps(
          CAMERA_RESULTS
            .stream()
            .mapToDouble(PhotonPipelineResult::getTimestampSeconds)
            .toArray()
        );
        CAMERA_RESULTS.clear();
      }
    }
  }
}