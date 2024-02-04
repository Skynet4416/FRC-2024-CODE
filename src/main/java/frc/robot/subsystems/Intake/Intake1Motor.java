package frc.robot.subsystems.Intake;

import java.util.Optional;
import frc.robot.Robot;
import frc.robot.Constants.Intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class Intake1Motor extends SubsystemBase
{
    private final CANSparkMax m_motor;
    private final EncoderSim m_encoderSim = null;

    public Intake1Motor()
    {
        this.m_motor = new CANSparkMax(Intake.Motors.kUpperMotorID,CANSparkLowLevel.MotorType.kBrushless);

        if (m_encoderSim != null)
        {
           // m_encoderSim = new EncoderSim(m_motor.getEncoder());
        }
    }

    //does this subsystem need configuration? probably? maybe?

    public void SetInOpenLoop(double speed)
    {
        m_motor.set(speed);

    }

    public void SetInOpenLoopSim(double speed)
    {
        System.out.println(speed * 0.5);
    }
}