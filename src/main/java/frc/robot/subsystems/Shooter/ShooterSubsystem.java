package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase 
{
    private final CANSparkMax m_motor;
    
    public ShooterSubsystem()
    {
        this.m_motor = new CANSparkMax(Shooter.Motors.ShooterMotorID,CANSparkLowLevel.MotorType.kBrushless);
    }
    public void SetSpeed(double speed)
    {
        m_motor.set(speed);
    }
        public void setVoltage(double voltage)
    {
        m_motor.setVoltage(voltage);
    }
}