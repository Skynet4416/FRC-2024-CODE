// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.ArrayList;
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

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final ArrayList<VisionObserver> observers;

    public VisionSubsystem(ArrayList<VisionObserver> observers) {
        AprilTagFieldLayout layout = null;
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("cant load layout");
        }
        this.observers = observers;
        photonPoseEstimator = new PhotonPoseEstimator(layout, Constants.Vision.poseStrategy, Constants.Vision.transformCamera);
        photonCamera = new PhotonCamera(Constants.Vision.cameraName);
    }

    public boolean aprilTagHasTarget() {
        if (Constants.Vision.enable) {
            return photonCamera.getLatestResult().hasTargets();

        }
        return false;
    }

    public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (Vision.enable) {
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            Optional<EstimatedRobotPose> pos = photonPoseEstimator.update();
            if (pos.isPresent()) {
                return pos.get();
            } else {
                System.out.println("CAMERA DISCONNECTED!!!!");
            }
        }
        return null;

    }

    @Override
    public void periodic() {
        if (aprilTagHasTarget()) {
            for (VisionObserver observer : observers
            ) {
                observer.addVisionMeasurement(getEstimatedGlobalPose(observer.getCurrentPosition()));
            }
        }
    }


}