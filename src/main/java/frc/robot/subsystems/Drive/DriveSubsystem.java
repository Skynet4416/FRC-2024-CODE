// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Vision.VisionObserver;
import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Swerve.SwerveModule;


public class DriveSubsystem extends SubsystemBase implements VisionObserver {

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final AHRS m_navX;
    private final double m_navXOffset;
    private SwerveModulePosition[] m_modulePositions;
    private ChassisSpeeds m_swerveSpeeds;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d field2d = new Field2d();

    public SwerveModule get_fl() {
        return m_frontLeftModule;
    }

    public SwerveModule get_fr() {
        return m_frontRightModule;
    }

    public SwerveModule get_bl() {
        return m_backLeftModule;
    }

    public SwerveModule get_br() {
        return m_backRightModule;
    }


    public DriveSubsystem(Pose2d startPos) {
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
        m_modulePositions = getModulePositions();
        m_swerveSpeeds = new ChassisSpeeds(0, 0, 0);
        m_poseEstimator = new SwerveDrivePoseEstimator(Drive.Stats.kinematics, getGyroAngleInRotation2d(), m_modulePositions, startPos);
        m_navXOffset = m_navX.getCompassHeading();
    }

    /**
     * Sets the state of all of the swerve modules
     *
     * @param moduleState WPILib's SwerveModuleState library
     */
    public void setModulesStates(SwerveModuleState[] moduleState) {
        m_frontLeftModule.setModuleState(moduleState[0]);
        m_frontRightModule.setModuleState(moduleState[1]);
        m_backLeftModule.setModuleState(moduleState[2]);
        m_backRightModule.setModuleState(moduleState[3]);
    }

    /**
     * Returns the field oriented corrected velocity for a target velocity
     *
     * @param targetVelocityX The target X velocity (Meters Per Second)
     * @param targetVelocityY The target Y velocity (Meters Per Second)
     */
    public double getVelocityFieldOriented_X(double targetVelocityX, double targetVelocityY) {
        double offsetAngle = getGyroAngleInRotation2d().getDegrees() - Drive.Stats.fieldHeadingOffset;
        double corrected_velocity = targetVelocityX * Math.cos(Math.toRadians(offsetAngle)) - targetVelocityY * Math.sin(Math.toRadians(offsetAngle));
        return corrected_velocity;
    }

    /**
     * Returns the field oriented corrected velocity for a target velocity
     *
     * @param targetVelocityX The target X velocity (Meters Per Second)
     * @param targetVelocityY The target Y velocity (Meters Per Second)
     */
    public double getVelocityFieldOriented_Y(double targetVelocityX, double targetVelocityY) {
        double offsetAngle = getGyroAngleInRotation2d().getDegrees() - Drive.Stats.fieldHeadingOffset;
        double corrected_velocity = targetVelocityX * Math.sin(Math.toRadians(offsetAngle)) + targetVelocityY * Math.cos(Math.toRadians(offsetAngle));
        return corrected_velocity;
    }

    /**
     * Sets the Speed / Angle / Stats of all of the modules
     *
     * @param xVelocityMps        The X velocity (Meters Per Second)
     * @param yVelocityMps        The Y velocity (Meters Per Second)
     * @param rotationVelocityRps Rotation velocity (Radians Per Second)
     */
    public void setModules(double xVelocityMps, double yVelocityMps, double rotationVelocityRps) {
        // TODO oriented to object on field
        double xVelocityMpsFieldOriented = getVelocityFieldOriented_X(xVelocityMps, yVelocityMps);
        double yVelocityMpsFieldOriented = getVelocityFieldOriented_Y(xVelocityMps, yVelocityMps);

        this.m_swerveSpeeds = new ChassisSpeeds(-xVelocityMpsFieldOriented, -yVelocityMpsFieldOriented, -rotationVelocityRps * 1.2);
        SwerveModuleState[] target_states = Drive.Stats.kinematics.toSwerveModuleStates(this.m_swerveSpeeds);
        setModulesStates(target_states);
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
        double angleWithOffset = (double) m_navX.getFusedHeading() + m_navXOffset;
        // Bigger than 360: angleWithOffset - 360
        // Smaller than 0: angleWithOffset + 360
        return (angleWithOffset > 360) ? angleWithOffset - 360 : (angleWithOffset < 0) ? angleWithOffset + 360 : angleWithOffset;
    }

    @Override
    public void periodic() {
        m_modulePositions = getModulePositions();
        m_poseEstimator.update(getGyroAngleInRotation2d(), m_modulePositions);
        field2d.setRobotPose(m_poseEstimator.getEstimatedPosition());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                new SwerveModulePosition(m_frontLeftModule.getDriveDistance(), m_frontLeftModule.getSteerAngle()),
                new SwerveModulePosition(m_frontRightModule.getDriveDistance(), m_frontRightModule.getSteerAngle()),
                new SwerveModulePosition(m_backLeftModule.getDriveDistance(), m_backLeftModule.getSteerAngle()),
                new SwerveModulePosition(m_backRightModule.getDriveDistance(), m_backRightModule.getSteerAngle())
        };
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void addVisionMeasurement(EstimatedRobotPose pos) {
        m_poseEstimator.addVisionMeasurement(pos.estimatedPose.toPose2d(), pos.timestampSeconds);
    }

    @Override
    public Pose2d getCurrentPosition() {
        return m_poseEstimator.getEstimatedPosition();
    }
}
