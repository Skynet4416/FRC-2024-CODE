// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ArmSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final CANSparkMax m_motor_left, m_motor_right;
    private final DutyCycleEncoder m_left_encoder;
    private final SparkPIDController m_Arm_Pid;
    private final RelativeEncoder m_Arm_Encoder;
    private static final double encoderOffset = 0.0;
    private static final double gearRatio = 1/25.0;

    public ArmSubsystem() {
        m_motor_left = new CANSparkMax(Constants.Arm.Motors.kLeftMotorID,MotorType.kBrushless);
        m_motor_right = new CANSparkMax(Constants.Arm.Motors.kRightMotorID, MotorType.kBrushless);
        DigitalInput Left_encoder_input = new DigitalInput(Constants.Arm.Encoders.kLeftEncoderID);
        m_motor_right.follow(m_motor_left);
        m_left_encoder = new DutyCycleEncoder(Left_encoder_input);
        m_Arm_Pid = m_motor_left.getPIDController();
        m_Arm_Pid.setP(Constants.Arm.Pid.kP);
        m_Arm_Pid.setI(Constants.Arm.Pid.kI);
        m_Arm_Pid.setD(Constants.Arm.Pid.kD);
        m_Arm_Encoder = m_motor_left.getEncoder();
    }
    public void init() {
        double actual = (m_left_encoder.getAbsolutePosition() - encoderOffset)*gearRatio;
        m_Arm_Encoder.setPosition(actual);
    }
    /**
     * 
     * Example command factory method.
     *
     * @return a command
     */
    public void SetAngle(double TargetAngle)
    {
        m_Arm_Pid.setReference(TargetAngle, ControlType.kPosition);
        
    }
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
