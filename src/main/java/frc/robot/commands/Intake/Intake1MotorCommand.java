package frc.robot.commands.Intake;

import frc.robot.Constants;
import frc.robot.Constants.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake1Motor;

public class Intake1MotorCommand extends Command
{
    private final Intake1Motor m_intake;

    public Intake1MotorCommand(Intake1Motor intake)
    {
        this.m_intake = intake;

        addRequirements(intake);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        this.m_intake.SetInOpenLoop(Intake.Stats.kIntakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    //should we have something in there? maybe
    @Override
    public boolean isFinished() {
        return false;
    }
}