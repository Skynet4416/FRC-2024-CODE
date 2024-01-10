     // Copyright (c) FIRST and other WPILib contributors.
     // Open Source Software; you can modify and/or share it under the terms of
     // the WPILib BSD license file in the root directory of this project.
     package frc.robot.subsystems.Drive;

     import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

     import edu.wpi.first.math.controller.PIDController;
     import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
     import edu.wpi.first.math.geometry.Pose2d;
     import edu.wpi.first.math.geometry.Rotation2d;
     import edu.wpi.first.math.kinematics.ChassisSpeeds;
     import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
     import edu.wpi.first.math.kinematics.SwerveModulePosition;
     import edu.wpi.first.math.kinematics.SwerveModuleState;
     import edu.wpi.first.math.util.Units;
     import edu.wpi.first.wpilibj2.command.SubsystemBase;
     import frc.robot.Constants;
     import frc.robot.Constants.Drive;
     import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Vision.VisionSubsystem;


     public class DriveSubsystem extends SubsystemBase {
     // TODO https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner
     // TODO https://github.com/CrossTheRoadElec/SwerveDriveExample/blob/main/src/main/java/frc/robot/CTRSwerve/CTRSwerveModule.java
     private final SwerveModule m_frontLeftModule;
     private final SwerveModule m_frontRightModule;
     private final SwerveModule m_backLeftModule;
     private final SwerveModule m_backRightModule;
     private final AHRS m_navX;
     private final double m_navXoffset;
     private SwerveModulePosition[] m_modulePositions;
     private final SwerveDriveOdometry m_odometry;
     private final PIDController m_pidController;
     private ChassisSpeeds m_swerveSpeeds;
     private Pose2d m_lastPose;
     private VisionSubsystem m_visionSubsystem;
     private Pose2d m_currentPose;
     private SwerveDrivePoseEstimator m_poseEstimator;
     private double m_targetAngle;

     public SwerveModule get_fl(){
          return m_frontLeftModule;
     }
     public SwerveModule get_fr(){
          return m_frontRightModule;
     }
     public SwerveModule get_bl(){
          return m_backLeftModule;
     }
     public SwerveModule get_br(){
          return m_backRightModule;
     }



     public DriveSubsystem() {
          this.m_frontLeftModule = new SwerveModule(
               Drive.Motors.kFrontLeftDriveFalconCANID, 
               Drive.Motors.kFrontLeftSteerFalconCANID, 
               Drive.Encoders.kFrontLeftSteerEncoderCANID, 
               Drive.Stats.kFrontLeftModuleOffsetInDegrees
          );
          this.m_frontRightModule = new SwerveModule(
               Drive.Motors.kFrontRightDriveFalconCANID, 
               Drive.Motors.kFrontRightSteerFalconCANID, 
               Drive.Encoders.kFrontRightSteerEncoderCANID,
               Drive.Stats.kFrontRightModuleOffsetInDegrees
               );
          this.m_backLeftModule = new SwerveModule(
               Drive.Motors.kBackLeftDriveFalconCANID, 
               Drive.Motors.kBackLeftSteerFalconCANID, 
               Drive.Encoders.kBackLeftSteerEncoderCANID,
               Drive.Stats.kBackLeftModuleOffsetInDegrees
          );
          this.m_backRightModule = new SwerveModule(
               Drive.Motors.kBackRightDriveFalconCANID, 
               Drive.Motors.kBackRightSteerFalconCANID, 
               Drive.Encoders.kBackRightSteerEncoderCANID,
               Drive.Stats.kBackRightModuleOffsetInDegrees
          );


          m_navX = new AHRS();
          

          m_modulePositions = new SwerveModulePosition[]{ 
               new SwerveModulePosition(m_frontLeftModule.getVelocityMetersPerSecond(), m_frontLeftModule.getSteerAngle()), 
               new SwerveModulePosition(m_frontRightModule.getVelocityMetersPerSecond(), m_frontRightModule.getSteerAngle()),
               new SwerveModulePosition(m_backLeftModule.getVelocityMetersPerSecond(), m_backLeftModule.getSteerAngle()), 
               new SwerveModulePosition(m_backRightModule.getVelocityMetersPerSecond(), m_backRightModule.getSteerAngle())
          };
               


          m_odometry = new SwerveDriveOdometry(Drive.Stats.kinematics, Rotation2d.fromDegrees(getHeading()), m_modulePositions); 
          
          m_swerveSpeeds = new ChassisSpeeds(0, 0, 0);

          m_currentPose = m_odometry.getPoseMeters(); // TODO needs to take the position from vision 
          m_pidController = new PIDController(Drive.PID.kP, Drive.PID.kI, Drive.PID.kD);
          m_pidController.enableContinuousInput(0, 360);
          m_poseEstimator = new SwerveDrivePoseEstimator(Drive.Stats.kinematics, getGyroAngleInRotation2d(), m_modulePositions, m_currentPose);
          m_lastPose = m_poseEstimator.getEstimatedPosition();
          m_navXoffset = (double)m_navX.getCompassHeading();
          m_visionSubsystem = new VisionSubsystem();
          m_targetAngle = 0.0;
     }
     /**
      * Sets the state of all of the swerve modules
      * @param moduleState
      * WPILib's SwerveModuleState library
      */
     public void setModulesStates(SwerveModuleState[] moduleState) {
          m_frontLeftModule.setModuleState(moduleState[0]);
          m_frontRightModule.setModuleState(moduleState[1]);
          m_backLeftModule.setModuleState(moduleState[2]);
          m_backRightModule.setModuleState(moduleState[3]);
     }

     /**
      * Returns the field oriented corrected velocity for a target velocity
      * @param targetVelocityX
      * The target X velocity (Meters Per Second)
      * @param targetVelocityY
      * The target Y velocity (Meters Per Second)
      */
     public double getVelocityFieldOriented_X(double targetVelocityX, double targetVelocityY){
          double offsetAngle = getGyroAngleInRotation2d().getDegrees() - Drive.Stats.fieldHeadingOffset;
          double corrected_velocity = targetVelocityX * Math.cos(Math.toRadians(offsetAngle)) - targetVelocityY*Math.sin(Math.toRadians(offsetAngle));
          return corrected_velocity;
     }

     /**
      * Returns the field oriented corrected velocity for a target velocity
      * @param targetVelocityX
      * The target X velocity (Meters Per Second)
      * @param targetVelocityY
      * The target Y velocity (Meters Per Second)
      */
     public double getVelocityFieldOriented_Y(double targetVelocityX, double targetVelocityY){
          double offsetAngle = getGyroAngleInRotation2d().getDegrees() - Drive.Stats.fieldHeadingOffset;
          double corrected_velocity = targetVelocityX * Math.sin(Math.toRadians(offsetAngle)) + targetVelocityY*Math.cos(Math.toRadians(offsetAngle));
          return corrected_velocity;
     }

     /**
      * Sets the Speed / Angle / Stats of all of the modules
      * @param xVelocityMps
      * The X velocity (Meters Per Second)
      * @param yVelocityMps
      * The Y velocity (Meters Per Second)
      * @param rotationVelocityRps
      * Rotation velocity (Radians Per Second)
      */
     public void setModules(double xVelocityMps, double yVelocityMps, double rotationVelocityRps) {
          // TODO oriented to object on field
          // m_targetAngle += Units.radiansToDegrees(rotationVelocityRps ) * 0.02;
          double xVelocityMpsFieldOriented = getVelocityFieldOriented_X(xVelocityMps,yVelocityMps);
          double yVelocityMpsFieldOriented = getVelocityFieldOriented_Y(xVelocityMps,yVelocityMps);

          //   m_targetAngle = getGyroAngleInRotation2d().getDegrees();
               this.m_swerveSpeeds = new ChassisSpeeds(-xVelocityMpsFieldOriented, -yVelocityMpsFieldOriented, -rotationVelocityRps * 1.2);

               //important! this is the pid controller that will be turned off during teleop but on during auto
               // this.m_swerveSpeeds = new ChassisSpeeds(-xVelocityMpsFieldOriented, -yVelocityMpsFieldOriented, -m_pidController.calculate(this.getGyroAngleInRotation2d().getDegrees()));
               // m_pidController.setSetpoint(m_targetAngle);



          SwerveModuleState[] target_states = Drive.Stats.kinematics.toSwerveModuleStates(this.m_swerveSpeeds);
          setModulesStates(target_states);
     }


     public void resetOdometry() {
          m_odometry.resetPosition(m_navX.getRotation2d(), m_modulePositions, m_currentPose);
     }

     public void setAllModulesToZero() {
          SwerveModuleState[] zeroStates = new SwerveModuleState[4];

          zeroStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.kFrontLeftModuleOffsetInDegrees));
          zeroStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.kFrontRightModuleOffsetInDegrees));
          zeroStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.kBackLeftModuleOffsetInDegrees));
          zeroStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.kBackRightModuleOffsetInDegrees));

          setModulesStates(zeroStates);
     }

          /**
           * gets the angle of the navx 
          */
     public Rotation2d getGyroAngleInRotation2d() {
          return Rotation2d.fromDegrees(getHeading());
     }

     public double getHeading() {
          double angleWithOffset = (double)m_navX.getFusedHeading() + m_navXoffset;
          // Bigger than 360: angleWithOffset - 360
          // Smaller than 0: angleWithOffset + 360
          return (angleWithOffset > 360) ? angleWithOffset - 360 : (angleWithOffset < 0) ? angleWithOffset + 360 : angleWithOffset;
     }

     public void addVisionMeasurements(Optional<EstimatedRobotPose> visionOptionalPose) {
          if (visionOptionalPose.isPresent()) {
               EstimatedRobotPose pose = visionOptionalPose.get();
               m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
          }
     }

     @Override
     public void periodic()
     {
          m_modulePositions = new SwerveModulePosition[]{ 
               new SwerveModulePosition(m_frontLeftModule.getVelocityMetersPerSecond(), m_frontLeftModule.getSteerAngle()), 
               new SwerveModulePosition(m_frontRightModule.getVelocityMetersPerSecond(), m_frontRightModule.getSteerAngle()),
               new SwerveModulePosition(m_backLeftModule.getVelocityMetersPerSecond(), m_backLeftModule.getSteerAngle()), 
               new SwerveModulePosition(m_backRightModule.getVelocityMetersPerSecond(), m_backRightModule.getSteerAngle())
          };
          m_odometry.update(getGyroAngleInRotation2d(), m_modulePositions);

          SwerveModuleState[] states = Drive.Stats.kinematics.toSwerveModuleStates(m_swerveSpeeds);

          m_frontLeftModule.setModuleState(states[0]);
          m_frontRightModule.setModuleState(states[1]);
          m_backLeftModule.setModuleState(states[2]);
          m_backRightModule.setModuleState(states[3]);

          m_poseEstimator.update(getGyroAngleInRotation2d(), m_modulePositions);
          addVisionMeasurements(m_visionSubsystem.getEstimatedGlobalPose(m_lastPose));
          m_lastPose = m_poseEstimator.getEstimatedPosition();  
     }


     @Override
     public void simulationPeriodic() {
          // This method will be called once per scheduler run during simulation
     }
}
