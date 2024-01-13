package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;

public interface VisionObserver {
    void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose);

    Pose2d getCurrentPosition();
}
