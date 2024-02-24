// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AllRobot;
import frc.robot.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor_left, m_motor_right;
    private final DutyCycleEncoder m_encoder;

    private PIDController pidController = new PIDController(Arm.Pid.kP, Arm.Pid.kD, Arm.Pid.kI);

    public ArmSubsystem() {
        m_motor_left = new CANSparkMax(Arm.Motors.kLeftMotorID, MotorType.kBrushless);
        m_motor_right = new CANSparkMax(Arm.Motors.kRightMotorID, MotorType.kBrushless);
        m_motor_left.restoreFactoryDefaults();
        m_motor_right.restoreFactoryDefaults();

        m_encoder = new DutyCycleEncoder(Arm.Encoders.kLeftEncoderID);

        m_motor_left.setSmartCurrentLimit(AllRobot.kAllMotorsLimitInAmpr);
        m_motor_right.setSmartCurrentLimit(AllRobot.kAllMotorsLimitInAmpr);
        m_motor_left.setIdleMode(IdleMode.kBrake);
        m_motor_right.setIdleMode(IdleMode.kBrake);

        m_motor_right.follow(m_motor_left, true);
    }

    public void setVoltage(double leftVoltage, double rightVoltage) {
        // No need to set right, as it follows left.
        m_motor_left.setVoltage(leftVoltage);
    }

    /**
     * 
     * Example command factory method.
     *
     * @return a command
     */
    public void SetAngle(double TargetAngle) {
        pidController.setSetpoint(TargetAngle);

    }

    public double getAngle() {
        return m_encoder.getAbsolutePosition() - Arm.Stats.encoderOffset;
    }

    public void execute() {
        double voltage = pidController.calculate(getAngle());
        setVoltage(voltage, -voltage);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
