package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmCommand extends Command {
    private ArmSubsystem armSubsystem;

    public ArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.resetAngle();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        armSubsystem.execute();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setVoltage(0);
    }
}
