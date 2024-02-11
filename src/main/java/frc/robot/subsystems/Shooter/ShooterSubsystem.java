package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase 
{
    private final CANSparkMax m_motor_left,m_motor_right;
    private final PIDController m_Shooter_Pid;

    public ShooterSubsystem()
    {
        this.m_motor_left = new CANSparkMax(Shooter.Motors.ShooterMotorLeftID,CANSparkLowLevel.MotorType.kBrushless);
        this.m_motor_right = new CANSparkMax(Shooter.Motors.ShooterMotorRightID,CANSparkLowLevel.MotorType.kBrushless);
        this.m_Shooter_Pid = new PIDController(Shooter.PID.kP,Shooter.PID.kI,Shooter.PID.kD);
    }
    public void SetSpeed(double speed)
    {
        m_motor_left.set(m_Shooter_Pid.calculate(m_motor_right.getAppliedOutput(),speed));
        m_motor_right.set(m_Shooter_Pid.calculate(m_motor_right.getAppliedOutput(),speed*-1));
    }
        public void setVoltage(double voltage)
    {
        m_motor_right.setVoltage(voltage);
        m_motor_left.setVoltage(voltage);
    }
}