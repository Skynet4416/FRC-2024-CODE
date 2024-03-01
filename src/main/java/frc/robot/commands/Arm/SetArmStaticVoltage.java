package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class SetArmStaticVoltage extends Command {
    private ArmSubsystem armSubsystem;
    private double voltage;

    public SetArmStaticVoltage(ArmSubsystem armSubsystem, double voltage) {
        this.armSubsystem = armSubsystem;
        this.voltage = voltage;

        // Requirement so SetArmVoltage can end when another begins.
        addRequirements(this.armSubsystem);
    }

    @Override
    public void initialize() {
        this.armSubsystem.setStaticVoltage(this.voltage);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
