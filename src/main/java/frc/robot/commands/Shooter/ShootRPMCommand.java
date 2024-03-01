package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShootRPMCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private double RPM;

    public ShootRPMCommand(ShooterSubsystem shooterSubsystem, double RPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.RPM = RPM;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.SetRPM(this.RPM);
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
