package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class ClimbCommand extends Command 
{
    public final ClimberSubsystem m_ClimberSubsystem;

    public ClimbCommand(ClimberSubsystem climberSubsystem)
    {
        this.m_ClimberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        m_ClimberSubsystem.extendTelescope();
        m_ClimberSubsystem.retractTelescope();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        if (m_ClimberSubsystem.isOpen()) 
        {
            return true;
        }
        return false;
    }
}
