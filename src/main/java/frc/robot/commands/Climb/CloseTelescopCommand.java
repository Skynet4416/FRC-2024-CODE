package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class CloseTelescopCommand extends Command {
    private final ClimberSubsystem m_ClimberSubsystem;

    public CloseTelescopCommand(ClimberSubsystem climberSubsystem) {
        this.m_ClimberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_ClimberSubsystem.retractTelescope();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    //TODO: end the command when climb is retracted
    @Override
    public boolean isFinished() {
        return true;
    }
}
