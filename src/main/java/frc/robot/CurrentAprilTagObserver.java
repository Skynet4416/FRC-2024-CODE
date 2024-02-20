package frc.robot;

import org.photonvision.targeting.PhotonTrackedTarget;

public interface CurrentAprilTagObserver {
    public void updateAprilTag(PhotonTrackedTarget aprilTag);
}
