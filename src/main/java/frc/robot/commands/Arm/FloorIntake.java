package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class FloorIntake extends Command {
    private final ArmSubsystem ArmSubsystem;
    public FloorIntake(ArmSubsystem ArmSubsystem)
    {
        this.ArmSubsystem = ArmSubsystem;
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
        ArmSubsystem.SetAngle(0.0);
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
