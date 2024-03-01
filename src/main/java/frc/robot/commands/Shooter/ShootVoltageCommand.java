package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShootVoltageCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private double m_voltage;

    public ShootVoltageCommand(ShooterSubsystem shooterSubsystem,
            double voltage) {
        this.shooterSubsystem = shooterSubsystem;
        this.m_voltage = voltage;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setVoltage(this.m_voltage);
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setVoltage(0);
    }
}
