package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Autonomous;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.controllers.*;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.util.*;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.*;
import com.pathplanner.lib.commands.FollowPathHolonomic;

import frc.robot.subsystems.Drive.DriveSubsystem;

//path planner's docs : https://pathplanner.dev/pplib-follow-a-single-path.html
public class Auto implements Subsystem {
    public DriveSubsystem m_drive;

    public Auto(DriveSubsystem drive) {
        this.m_drive = drive;
        // all the comments inside the function are from the pathplanner docs, don't
        // delete them until all the parematers are set
        AutoBuilder.configureHolonomic(
                () -> {
                    return m_drive.getCurrentPose();
                }, // Robot pose supplier
                (Pose2d currentPose) -> {
                    m_drive.resetOdometry(currentPose);
                }, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> {
                    return m_drive.getSwerveSpeeds();
                }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (ChassisSpeeds speeds) -> {
                    m_drive.setModules(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond,0.0);
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                Autonomous.kHolonomicPathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this.m_drive);
    }

    /**
     * @param pathName
     * @return
     */
    // this function just magically stopped being an error???? i don't know what did
    // it????

    public Command followPathCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathHolonomic(
                path,
                () -> {
                    return m_drive.getCurrentPose();
                }, // Robot pose supplier
                () -> {
                    return m_drive.getSwerveSpeeds();
                }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (ChassisSpeeds speeds) -> {
                    m_drive.setModules(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond,0.0);
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                Autonomous.kHolonomicPathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this.m_drive // Reference to this subsystem to set requirements
        );
    }
}
