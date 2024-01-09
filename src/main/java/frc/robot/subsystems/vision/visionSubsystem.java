package frc.robot.subsystems.vision;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;

public class VisionSubsystem extends SubsystemBase
{
    private final PhotonCamera m_camera;
    boolean hasTarget; // Stores whether or not a target is detected
    PhotonPipelineResult result; // Stores all the data that Photonvision returns

    public VisionSubsystem()
    {
        this.m_camera = new PhotonCamera("Aizen");
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = m_camera.getLatestResult(); // Query the latest result from PhotonVision
        hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean will be true
        if (hasTarget) {
            this.result = result;
        }
    }
    public PhotonTrackedTarget getTargetWithID(int id) { // Returns the apriltag target with the specified ID (if it exists)
        List<PhotonTrackedTarget> targets = result.getTargets(); // Create a list of all currently tracked targets
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == id) { // Check the ID of each target in the list
                return i; // Found the target with the specified ID!
            }
        }
        return null; // Failed to find the target with the specified ID
    }
    
    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget) {
        return result.getBestTarget(); // Returns the best (closest) target
        }
        else {
            return null; // Otherwise, returns null if no targets are currently found
        }
    }
    public boolean getHasTarget() {
        return hasTarget; // Returns whether or not a target was found
    }
}
