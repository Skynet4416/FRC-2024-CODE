package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Vision.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.InRangeObserver;
import frc.robot.Constants.Vision;

public class VisionObserverCentral implements VisionObserver 
{
    private Pose2d m_currentPose;
    private InRangeObserver inRange;
    
    public VisionObserverCentral(Pose2d m_currentPose, InRangeObserver inRange) 
    {
        this.inRange = inRange;
        this.m_currentPose = m_currentPose;
    }

    @Override
     public void hasNoTags()
     { }

    @Override
    public Pose2d getCurrentPosition()
    {
        return m_currentPose;
    }
    
     @Override
     public void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose, PhotonTrackedTarget aprilTag)
     {
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red && aprilTag.getFiducialId()==Vision.AprilTags.aprilTagIDRed)
        {
            
        }
        else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue && aprilTag.getFiducialId()==Vision.AprilTags.aprilTagIDBlue)
        {
            
        }
     }
}
