package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;

public interface VisionObserver {
    void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose);

    /**
     * if the camera found no april tag than this is a function that is used to delete previous information saved about april tags of intrest. if you are reading this, i have not finised the part that saves information yet
     */
    void hasNoTags();

    Pose2d getCurrentPosition();
}