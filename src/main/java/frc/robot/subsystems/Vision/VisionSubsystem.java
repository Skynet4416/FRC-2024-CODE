// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera m_camera;
    private AprilTagFieldLayout m_layout;
    private SwerveDrivePoseEstimator m_poseEstimator;
    private PhotonPipelineResult m_result;
// }

//     public VisionSubsystem() {
//         this.m_camera = new PhotonCamera("aizen");
//         this.m_result = getLatestResult();
//         try {
//             this.m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
//         } catch (IOException exception) {}
//         this.m_poseEstimator = new PhotonPoseEstimator(m_layout, PoseStrategy.AVERAGE_BEST_TARGETS, Vision.transformCamera);

//     }
//      //TODO
//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
//         m_poseEstimator.setReferencePose(prevEstimatedRobotPose);
//         return updatePoseEstimator();
//     }

//     public void updatePoseEstimator()
//     {
//         m_poseEstimator.update(getHeading(), SwerveModulePosition)
//         var res = m_camera.getLatestResult();
//         if (res.hasTargets()) {
//             var imageCaptureTime = res.getTimestampSeconds();
//             var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
//             var camPose = Vision.kFarTargetPose.transformBy(camToTargetTrans.inverse());
//             m_poseEstimator.addVisionMeasurement(
//                     camPose.transformBy(Vision.transformCamera).toPose2d(), imageCaptureTime);
//     }

//     public Optional<PhotonTrackedTarget> getTargetById(int id) {
//         List<PhotonTrackedTarget> targets = m_result.getTargets();
//         for (PhotonTrackedTarget target : targets) {
//             if (target.getFiducialId() == id) { 
//                 return Optional.of(target);
//             }
//         }
//         return Optional.empty();
//     }

//     public void addVisionMeasurements(Optional<EstimatedRobotPose> visionOptionalPose) {
//         if (visionOptionalPose.isPresent()) {
//              EstimatedRobotPose pose = visionOptionalPose.get();
//              m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
//         }
//    }

//     public PhotonPipelineResult getLatestResult() {
//         return m_camera.getLatestResult();
//     }

//     public boolean hasTarget() {
//         return m_camera.getLatestResult().hasTargets();
//     }

//     public PhotonTrackedTarget getTarget() {
//         return m_camera.getLatestResult().getBestTarget();
//     }

//     public void resetOdometry() {
//         m_odometry.resetPosition(m_navX.getRotation2d(), m_modulePositions, m_currentPose);
// }
//     @Override
//     public void periodic() {
//         //this really should be elsewhere
//         m_odometry.update(getGyroAngleInRotation2d(), m_modulePositions);
//           SwerveModuleState[] states = Drive.Stats.kinematics.toSwerveModuleStates(m_swerveSpeeds);

//         m_result = getLatestResult();

//         m_poseEstimator.update(getGyroAngleInRotation2d(), m_modulePositions);
//           addVisionMeasurements(m_visionSubsystem.getEstimatedGlobalPose(m_lastPose));
//           m_lastPose = m_poseEstimator.getEstimatedPosition();  
//         // TODO
//     }

//     @Override
//     public void simulationPeriodic() {
//         // This method will be called once per scheduler run during simulation
//     }
 }