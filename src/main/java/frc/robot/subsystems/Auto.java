package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.DriveSubsystem;
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
public class Auto implements Subsystem 
{
    public DriveSubsystem m_drive;
    public Auto(DriveSubsystem drive)
    {
        this.m_drive = drive;
        //all the comments inside the function are from the pathplanner docs, don't delete them until all the parematers are set 
        //this function is for a subsystem, which is why it is an error in a non subsystem class   
        AutoBuilder.configureHolonomic(
                () -> {return m_drive.getCurrentPose();}, // Robot pose supplier
                (Pose2d m_currentPose)-> { m_drive.resetOdometry();}, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> {return m_drive.getSwerveSpeeds();}, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (ChassisSpeeds speeds)->{m_drive.setModules(0, 0, 0);}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }
     /**
     * @param pathName
     * @return
     */
    //i have no idea why this function is such a hot mess for no reason. i couldn't figure out why nothing worked. guess it's a todo

//     public Command followPathCommand(String pathName) {
//         PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

//         return new FollowPathHolonomic(
//                 path,
//                 () -> {return m_drive.getCurrentPose();}, // Robot pose supplier
//                 () -> {return m_drive.getSwerveSpeeds();}, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//                 (ChassisSpeeds speeds)->{m_drive.setModules(0, 0, 0);}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//                 new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
//                         new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                         new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
//                         4.5, // Max module speed, in m/s
//                         0.4, // Drive base radius in meters. Distance from robot center to furthest module.
//                         new ReplanningConfig() // Default path replanning config. See the API for the options here
//                 ),
//                 () -> {
//                     // Boolean supplier that controls when the path will be mirrored for the red alliance
//                     // This will flip the path being followed to the red side of the field.
//                     // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//                     var alliance = DriverStation.getAlliance();
//                     if (alliance.isPresent()) {
//                         return alliance.get() == DriverStation.Alliance.Red;
//                     }
//                     return false;
//                 },
//                 this // Reference to this subsystem to set requirements
//         );
//     }
 }
