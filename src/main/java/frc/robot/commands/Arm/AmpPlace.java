package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class AmpPlace extends Command{
    private final ArmSubsystem m_ArmSubsystem;
    public AmpPlace(ArmSubsystem ArmSubsystem)
    {
        this.m_ArmSubsystem = ArmSubsystem;
        addRequirements(ArmSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        m_ArmSubsystem.SetAngle(Arm.Stats.ampAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
