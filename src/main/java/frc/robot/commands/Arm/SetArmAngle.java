package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class SetArmAngle extends Command {
    private ArmSubsystem armSubsystem;
    private double angle;

    public SetArmAngle(ArmSubsystem armSubsystem, double angle) {
        this.armSubsystem = armSubsystem;
        this.angle = angle;

        // This command does NOT require armSubsystem, it only sets its angle.
    }

    @Override
    public void initialize() {
        this.armSubsystem.SetAngle(this.angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
