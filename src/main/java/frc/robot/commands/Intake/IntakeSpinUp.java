package frc.robot.commands.Intake;

import frc.robot.Constants;
import frc.robot.Constants.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeSpinUp extends Command
{
    private final IntakeSubsystem m_intake;
    private boolean reversed;

    // Called when the command is initially scheduled.
    public IntakeSpinUp(IntakeSubsystem Intake,boolean reversed)
    {
        this.m_intake = Intake;
        addRequirements(Intake);
    }


    @Override
    public void initialize() {}

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
        return false;
    }
}