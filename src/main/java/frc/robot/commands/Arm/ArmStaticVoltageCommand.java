package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmStaticVoltageCommand extends Command {
    private ArmSubsystem armSubsystem;
    private double voltage;

    public ArmStaticVoltageCommand(ArmSubsystem armSubsystem, double voltage) {
        this.armSubsystem = armSubsystem;
        this.voltage = voltage;

        // Requirement so SetArmVoltage can end when another begins.
        addRequirements(this.armSubsystem);
    }

    @Override
    public void initialize() {
        this.armSubsystem.pidEnabled = false;
        this.armSubsystem.setVoltage(this.voltage);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
