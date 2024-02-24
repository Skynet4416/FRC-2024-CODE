// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.util.*;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.path.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Arm {
        public static class Motors {
            public static final int kLeftMotorID = 0;
            public static final int kRightMotorID = 0;
        }

        public static class Stats {
            public static final double encoderOffset = 0.0;
            public static final double gearRatio = 1 / 25.0;
            public static final double ampAngle = 110;
            public static final double speakerAngle = 10;
            public static final double driveAngle = 40;
            public static final double kThreashold = 0;
            public static final double kIntakeAngle = 0;
            public static final double climbAngle = 0;

        }

        public static class Encoders {
            public static final int kLeftEncoderID = 0;
            public static final int kRightEncoderID = 0;
        }

        public static class Pid {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }

    }

    public static class Swerve {

        public static class PID {
            // ! --- DO NOT USE THESE PID k VARIABLES IN PRODUCTION! I DID NOT TEST THEM YET
            // ------------------
            public static class Drive {
                /**
                 * Static Friction Offset (to overcome the friction of the system)
                 */
                public static final double kS = 0.0;
                /**
                 * Velocity Feedforward (to continue the current speed)
                 */
                public static final double kV = 0.0;
                /**
                 * the voltage needed to reach a certain acceleration (i have no idea what
                 * number to put)
                 */
                public static final double kA = 0.0;

                /**
                 * Proportional tuning - error
                 * Lower the kP
                 */

                public static final double kP = 0.0005;
                /**
                 * Integral tuning - learning
                 */
                public static final double kI = 0.0;
                /**
                 * Derivative tuning - overshoot
                 */
                public static final double kD = 0.0;
            }

            public static class Steer {
                /**
                 * Static Friction Offset (to overcome the friction of the system)
                 */
                public static final double kS = 0.0;
                /**
                 * Velocity Feedforward (to continue the current speed)
                 */
                public static final double kV = 0.0;
                /**
                 * the voltage needed to reach a certain acceleration (i have no idea what
                 * number to put)
                 */
                public static final double kA = 0.0;

                /**
                 * Proportional tuning - error
                 */
                public static final double kP = 10.5;
                /**
                 * Integral tuning - learning
                 */
                public static final double kI = 0.0;
                /**
                 * Derivative tuning - overshoot
                 */
                public static final double kD = 0.0;
            }

        }

        public static class Stats {
            // todo: change to the actual ratio of the neo vortex
            public static final double kVoltsPerRPM = 0;

            public static final double kMaxVoltage = 12.0;
            public static final double kStatorCurrentLimit = 35.0;
            public static final double kSupplyCurrentLimit = 35.0;

            /**
             * Distance between the center of the right wheels to the center of the left
             * wheels (Meters)
             */
            public static final double kTrackWidthMeters = 85.5;

            /**
             * +
             * Distance between the center of the back wheels to the center of the front
             * wheels (Meters)
             */
            public static final double kWheelbaseMeters = 85.5;

            /**
             * The ratio between the Motor and the center wheel of the Swerve module (which
             * the CANcoder lies on)
             */
            public static final double kRotorToSensorRatioDrive = 8.14;
            public static final double kRotorToSensorRatioSteer = 150 / 7;

            public static final double kDriveWheelRadiusInches = 2;
            public static final double wheelRadiusMeters = Units.inchesToMeters(kDriveWheelRadiusInches);

        }
    }

    public static class Intake {
        public static class Motors {
            // also the id if there is only one motor
            public static final int kUpperMotorLeftID = 0;
            public static final int kUpperMotorRightID = 0;
        }

        public static class Stats {
            // todo: set the speed needed, and everything in constants honestly
            public static final double kIntakeSpeed = 0.2;
            public static final double kPushingNodeInRounds = 1;
            public static final double kShooterSpeed = -0.2;
        }
    }

    public static class Shooter {
        public static class Motors {
            public static final int ShooterMotorLeftID = 69;
            public static final int ShooterMotorRightID = 69;

        }

        public static class Stats {
            // todo: set the speed needed
            public static final double kIShooterSpeed = 0.0;
            public static final double kPutInAmpSpeed = 0.0;
        }

        public static class PID {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static class Climber {
        public static class PID {
            // todo: set actual numbers
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static class Motors {
            public static final int kRightHookMotorID = 0;
            public static final int kLeftHookMotorID = 0;
        }

        public static class Stats {
            // the meters the climber gains when "open"
            public static final double kHeightChangeInMeters = 0.0;
            public static final double kMaxHeightInMeters = 0.0;
            // todo: change to actual numbers of stuff
            // the length we want the climber to climb (i think this is the addition to the
            // peropened telescope)
            public static final double kDesiredLength = 0.0;
            public static final double kSpoolCircumference = 0.0;
            // timsoret
            public static final double kGearRatio = 0.0;
            // the rounds the motor needs to do = length we want the climb to go/the
            // circumference of the spool*gear ratio
            public static final double kExtensionTurnsInRounds = kDesiredLength / kSpoolCircumference * kGearRatio;
            // the time it takes the telescop to extend at the chosen speed
            public static final long kExtendTimeMS = 0;
            // the time it takes the telescop to retract at the chosen speed
            public static final long kRetractTimeMS = 0;
            public static final double kThreashold = 0;
            public static final double kRetractInRounds = 0;
        }
    }

    public static class Drive {

        public static class Stats {
            public static final double fieldHeadingOffset = 0;

            /**
             * Distance between the center of the right wheels to the center of the left
             * wheels (Meters)
             */
            public static final double kTrackWidthMeters = 85.5;

            /**
             * Distance between the center of the back wheels to the center of the front
             * wheels (Meters)
             */
            public static final double kWheelbaseMeters = 85.5;

            /**
             * the distance of each module (assuming the drivetrain is a square) from the
             * center of the drivetrain. i made this for the auto. yes this pythagoram is
             * ugly, i know
             */
            public static final double kDriveBaseRadius = Math
                    .sqrt(Math.pow(kTrackWidthMeters / 2.0, 2.0) + Math.pow(kWheelbaseMeters / 2.0, 2.0));

            /**
             * The current degree of the steer mechanism (At what degree does the drive
             * wheel start)
             */
            public static final double kFrontLeftModuleOffsetInDegrees = -102;
            /**
             * The current degree of the steer mechanism (At what degree does the drive
             * wheel start)
             */
            public static final double kFrontRightModuleOffsetInDegrees = -156;
            /**
             * The current degree of the steer mechanism (At what degree does the drive
             * wheel start)
             */
            public static final double kBackLeftModuleOffsetInDegrees = -20;
            /**
             * The current degree of the steer mechanism (At what degree does the drive
             * wheel start)
             */
            public static final double kBackRightModuleOffsetInDegrees = 166;

            public static final double kMaxVelocityMetersPerSecond = 4.17576;
            public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond /
                    Math.hypot(kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0);

            public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                    // TODO needs to be configured with diffrent constants that has the modules
                    // position relative to the middle of the robot
                    new Translation2d(kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0), // ++
                    new Translation2d(kTrackWidthMeters / 2.0, -kWheelbaseMeters / 2.0), // +-
                    new Translation2d(-kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0), // -+
                    new Translation2d(-kTrackWidthMeters / 2.0, -kWheelbaseMeters / 2.0) // --
            );

        }

        // what are these PID's for? like just general?
        public static class PID {
            public static final double kP = 0.002;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static class Motors {
            public static final int kFrontLeftDriveFalconCANID = 11;
            public static final int kFrontLeftSteerFalconCANID = 9;

            public static final int kFrontRightDriveFalconCANID = 14;
            public static final int kFrontRightSteerFalconCANID = 12;

            public static final int kBackLeftDriveFalconCANID = 20;
            public static final int kBackLeftSteerFalconCANID = 18;

            public static final int kBackRightDriveFalconCANID = 17;
            public static final int kBackRightSteerFalconCANID = 15;

        }

        public static class Encoders {
            // ? Only the steer encoder exists (seperate from the encoder inside of the
            // Falcon 500 because of ratio problems between the wheels of the swerve
            // modules)
            public static final int kFrontLeftSteerEncoderCANID = 10;
            public static final int kFrontRightSteerEncoderCANID = 13;
            public static final int kBackLeftSteerEncoderCANID = 19;
            public static final int kBackRightSteerEncoderCANID = 16;
        }

    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class OI {
        public static final int kXboxControllerPort = 0;
        public static final double kXboxcontrollerDrift = 0.0;
    }

    public static class Field {
        /**
         * The field's length in Meters (Found in page 21 of the 2024 game manual)
         */
        public static final double fieldLength = Units.inchesToMeters(651.25);
        /**
         * The field's width in Meters (Found in page 21 of the 2024 game manual)
         */
        public static final double fieldWidth = Units.inchesToMeters(323.25);
        /**
         * The width (and length) in meters of the AprilTags (Found in page 35 of the
         * 2024 game manual)
         */
        public static final double aprilTagWidth = Units.inchesToMeters(8.125);
    }

    // cool it's still here while the vision isn't.
    public static class Vision {
        public static class Stats {
            public static class Range {
                // todo: measure and put the actual numbers
                public static final double kSmallestDistanceCM = 0.0;
                public static final double kBiggestDistanceCM = 0.0;
                public static final double kSmallestAngleDeg = 0.0;
                public static final double kBiggestAngleDeg = 0.0;
            }

            public static final String cameraName = "Aizen";
            public static final Translation3d robotMiddleToCamera = new Translation3d(0, 0, 0);
            public static final Rotation3d angleCamera = new Rotation3d(0, 0, 0);
            public static final double CameraHeightInCM = 20;
            public static final Transform3d transformCamera = new Transform3d(robotMiddleToCamera, angleCamera);
            // this value isn't intentional and can be changed, i just copied this line from
            // photonVision's example
            public static final double targetHeightInCM = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19);
            // these variables are for the update of the pose estimator
            public static final double kFarTgtXPos = Units.feetToMeters(54);
            public static final double kFarTgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75)
                    - Units.inchesToMeters(48.0 / 2.0);
            public static final double kFarTgtZPos = (Units.inchesToMeters(98.19) - targetHeightInCM) / 2
                    + targetHeightInCM;
            public static final Pose3d kFarTargetPose = new Pose3d(
                    new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));

        }

        public static boolean enable = true;
        public static PoseStrategy poseStrategy;

        public static class AprilTags {
            public static final int aprilTagIDRed = 4;
            public static final int aprilTagIDBlue = 7;
        }
    }

    public static class Autonomous {
        // untested values
        public static final double kLinearP = 0.1;
        public static final double kLinearI = 0.0;
        public static final double kLinearD = 0.0;

        public static final double kAngularP = 0.1;
        public static final double kAngularI = 0.0;
        public static final double kAngularD = 0.0;
        // i uncommented the i's for the auto (pathplanner wants all 3 values) please
        // don't recomment them. it might just send the auto to the twilight zone.

        public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(Autonomous.kLinearP, Autonomous.kLinearI, Autonomous.kLinearD),
                new PIDConstants(Autonomous.kAngularP, Autonomous.kAngularI, Autonomous.kAngularD),
                Drive.Stats.kMaxVelocityMetersPerSecond,
                Drive.Stats.kDriveBaseRadius, // Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        );
    }

    public static class AllRobot {
        public static final int kAllMotorsLimitInAmpr = 5;
    }
}