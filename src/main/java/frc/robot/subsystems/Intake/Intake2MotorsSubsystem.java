package frc.robot.subsystems.Intake;

import java.util.Optional;
import frc.robot.Robot;
import frc.robot.Constants.Intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake2MotorsSubsystem extends SubsystemBase
{
    private final CANSparkMax m_upperMotor;
    private final CANSparkMax m_lowerMotor;

    public Intake2MotorsSubsystem()
    {
        this.m_upperMotor = new CANSparkMax(Intake.Motors.kUpperMotorID,CANSparkLowLevel.MotorType.kBrushless);
        this.m_lowerMotor = new CANSparkMax(Intake.Motors.kLowerMotorID,CANSparkLowLevel.MotorType.kBrushless);
    }

    //does this subsystem need configuration? probably? maybe?

    public void SetInOpenLoop(double speed)
    {
        m_upperMotor.set(speed);
        m_lowerMotor.set(-speed);
    }
}