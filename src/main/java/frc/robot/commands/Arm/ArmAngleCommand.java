package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmAngleCommand extends Command {
    private ArmSubsystem armSubsystem;
    private double angle;

    public ArmAngleCommand(ArmSubsystem armSubsystem, double angle) {
        this.armSubsystem = armSubsystem;
        this.angle = angle;

        // Requirement so SetArmAngle can end when another begins.
        addRequirements(this.armSubsystem);
    }

    @Override
    public void initialize() {
        this.armSubsystem.pidEnabled = true;
        this.armSubsystem.setAngle(this.angle);
    }

    @Override
    public boolean isFinished() {
        // This is mostly for command groups, as there is no end function defined.
        return Math.abs(this.armSubsystem.getAngle() - this.angle) < Arm.Stats.kThreashold;
    }
}
