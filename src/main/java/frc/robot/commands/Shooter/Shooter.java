package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class Shooter extends Command{
    private final ShooterSubsystem m_shooter;

    public Shooter(ShooterSubsystem shooter)
    {
        this.m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        this.m_shooter.SetSpeed(Intake.Stats.kIntakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
        this.m_shooter.setVoltage(0);
    }

    // Returns true when the command should end.
    //should we have something in there? maybe
    @Override
    public boolean isFinished() {
        return false;
    }

}
