package frc.robot.commands.Climb;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class CloseClimbCommand extends Command {
    private ClimberSubsystem climberSubsystem;

    public CloseClimbCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.retractTelescope();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.isRetracted();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setVoltage(0);
    }
}
