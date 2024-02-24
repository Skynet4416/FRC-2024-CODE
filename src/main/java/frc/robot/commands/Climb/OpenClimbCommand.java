package frc.robot.commands.Climb;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class OpenClimbCommand extends Command {
    private ClimberSubsystem climberSubsystem;

    public OpenClimbCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.extendTelescope();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.isExtended();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setVoltage(0);
    }
}
