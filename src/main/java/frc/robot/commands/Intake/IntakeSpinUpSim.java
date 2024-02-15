package frc.robot.commands.Intake;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.math.system.plant.DCMotor;
import com.revrobotics.REVPhysicsSim;

public class IntakeSpinUpSim extends Command
{
    private final IntakeSubsystem m_intake;
    private boolean reversed;
    private int counter = 0;

    // Called when the command is initially scheduled.
    public IntakeSpinUpSim(IntakeSubsystem Intake,boolean reversed)
    {
        this.m_intake = Intake;
        addRequirements(Intake);
    }


    @Override
    public void initialize() 
    {
    REVPhysicsSim.getInstance().addSparkMax(m_intake.getMotorRight(), DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_intake.getMotorLeft(), DCMotor.getNEO(1));
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if(reversed)
        {
            this.m_intake.SetSpeed(Intake.Stats.kIntakeSpeed*-1);
        }
        else
        {   
            this.m_intake.SetSpeed(Intake.Stats.kIntakeSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
        this.m_intake.setVoltage(0);
    }

    // Returns true when the command should end.
    //should we have something in there? maybe
    @Override
    public boolean isFinished() {
        if (counter >= 3000)
        {
            counter = 0;
            return true;
        }
        counter++;
        
        return false;
    }
}