package frc.robot.subsystems.Intake;

import java.util.Optional;
import frc.robot.Robot;
import frc.robot.Constants.Intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase
{
    private final CANSparkMax m_motor_left,m_motor_right;

    public IntakeSubsystem()
    {
        this.m_motor_left = new CANSparkMax(Intake.Motors.kUpperMotorLeftID,CANSparkLowLevel.MotorType.kBrushless);
        this.m_motor_right = new CANSparkMax(Intake.Motors.kUpperMotorRightID,CANSparkLowLevel.MotorType.kBrushless);
    }

    //does this subsystem need configuration? probably? maybe?

    public void SetSpeed(double speed)
    {
        m_motor_left.set(speed);
        m_motor_right.set(speed*-1);
    }
    public void setVoltage(double voltage)
    {
        m_motor_left.setVoltage(voltage);
        m_motor_right.setVoltage(voltage);
    }
    public CANSparkMax getMotorRight()
    {
        return this.m_motor_right;
    }
    public CANSparkMax getMotorLeft()
    {
        return this.m_motor_left;
    }
}