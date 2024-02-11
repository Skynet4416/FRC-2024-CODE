// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ArmSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final CANSparkMax m_motor_left, m_motor_right;
    private final DutyCycleEncoder m_left_encoder, m_right_encoder;
    private final PIDController m_Arm_Pid;

    public ArmSubsystem() {
        this.m_motor_left = new CANSparkMax(Constants.Arm.Motors.kLeftMotorID,MotorType.kBrushless);
        this.m_motor_right = new CANSparkMax(Constants.Arm.Motors.kRightMotorID, MotorType.kBrushless);
        DigitalInput Left_encoder_input = new DigitalInput(Constants.Arm.Encoders.kLeftEncoderID);
        this.m_left_encoder = new DutyCycleEncoder(Left_encoder_input);
        DigitalInput Right_encoder_input = new DigitalInput(Constants.Arm.Encoders.kRightEncoderID);
        this.m_right_encoder = new DutyCycleEncoder(Right_encoder_input);
        this.m_Arm_Pid = new PIDController(Constants.Arm.Pid.kP, Constants.Arm.Pid.kI, Constants.Arm.Pid.kD);
    }

    
    
    
    
    
    
    public void SetAngle(double TargetAngle)
    {
        double avg_angle = (m_left_encoder.getAbsolutePosition() - m_right_encoder.getAbsolutePosition())/2;
        m_motor_left.set(m_Arm_Pid.calculate(avg_angle,TargetAngle));
        m_motor_right.set(m_Arm_Pid.calculate(avg_angle,TargetAngle));
        
    }

    
    /**
     * 
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
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
