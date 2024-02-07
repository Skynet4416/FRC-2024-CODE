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
    private final CANSparkMax m_motor;

    public IntakeSubsystem()
    {
        this.m_motor = new CANSparkMax(Intake.Motors.kUpperMotorID,CANSparkLowLevel.MotorType.kBrushless);
    }

    //does this subsystem need configuration? probably? maybe?

    public void SetSpeed(double speed)
    {
        m_motor.set(speed);
    }
    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);;
    }
}