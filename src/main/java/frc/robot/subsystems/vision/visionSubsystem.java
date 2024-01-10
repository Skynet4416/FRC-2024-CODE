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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera m_camera;
    private AprilTagFieldLayout m_layout;
    private PhotonPoseEstimator m_poseEstimator;
    private PhotonPipelineResult m_result;
    

    public VisionSubsystem() {
        this.m_camera = new PhotonCamera("aizen");
        this.m_result = getLatestResult();
        try {
            this.m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException exception) {}
        this.m_poseEstimator = new PhotonPoseEstimator(m_layout, PoseStrategy.AVERAGE_BEST_TARGETS, Vision.transformCamera);

    }
// TODO
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        m_poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_poseEstimator.update();
    }



    public Optional<PhotonTrackedTarget> getTargetById(int id) {
        List<PhotonTrackedTarget> targets = m_result.getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == id) { 
                return Optional.of(target);
            }
        }
        return Optional.empty();
    }

    public PhotonPipelineResult getLatestResult() {
        return m_camera.getLatestResult();
    }

    public boolean hasTarget() {
        return m_camera.getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getTarget() {
        return m_camera.getLatestResult().getBestTarget();
    }

    @Override
    public void periodic() {
        m_result = getLatestResult();
        // TODO
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}