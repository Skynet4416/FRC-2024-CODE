package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Vision.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.CurrentAprilTagObserver;
import frc.robot.InRangeObserver;
import frc.robot.Constants.Vision;

public class VisionObserverCentral implements VisionObserver {
    private Pose2d m_currentPose;
    private InRangeObserver inRange;
    private CurrentAprilTagObserver currentAprilTag;

    public VisionObserverCentral(Pose2d m_currentPose, InRangeObserver inRange) {
        this.inRange = inRange;
        this.m_currentPose = m_currentPose;
    }

    @Override
    public void hasNoTags() {
        inRange.inRange(false);
    }

    @Override
    public Pose2d getCurrentPosition() {
        return m_currentPose;
    }

    /**
     * not to be confused with the member inRange or the function inRange() within
     * the interface InRangeObserver, this function checks if the april tag is
     * within the required range (as in it checks the distance of the april tag and
     * it's angle).
     * 
     * @param aprilTag - the April tag that has already been identified as one that
     *                 represents the speaker
     * @return whether or not the april tag is in the range
     */
    private boolean checksIfInRange(PhotonTrackedTarget aprilTag) {
        double distance = VisionSubsystem.getDistanceInCM(aprilTag);
        return (aprilTag.getYaw() > Vision.Stats.Range.kSmallestAngleDeg
                && aprilTag.getYaw() < Vision.Stats.Range.kBiggestAngleDeg &&
                distance > Vision.Stats.Range.kSmallestDistanceCM && distance < Vision.Stats.Range.kBiggestDistanceCM);
    }

    /**
     * checks if the target is in the right range (depandened on the distance of the
     * bot from it and the yaw of the target compared to it), if it fulfills the
     * criteria it should send inRange with true, else with false
     */
    @Override
    public void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose, PhotonTrackedTarget aprilTag) {
        if ((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                && aprilTag.getFiducialId() == Vision.AprilTags.aprilTagIDBlue) ||
                (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                        && aprilTag.getFiducialId() == Vision.AprilTags.aprilTagIDRed)) {
            inRange.inRange(checksIfInRange(aprilTag));
            currentAprilTag.updateAprilTag(aprilTag);
            // send to dashboard april tag id and yaw and pitch
        } else {
            inRange.inRange(false);
        }
    }
}
