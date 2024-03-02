package frc.robot.commands.Shooter.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.Intake.IntakeNodeCommand;
import frc.robot.commands.Shooter.ShootAmpCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class SpeakerCommandGroup extends SequentialCommandGroup {
    ArmSubsystem armSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    // FIXME: Make group.
    public SpeakerCommandGroup(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;


        // addCommands(new ShootAmpCommand(shooterSubsystem, intakeSubsystem, null),
        //         new ArmCommand(armSubsystem, Arm.Stats.kIntakeAngle));

        addRequirements(armSubsystem, intakeSubsystem, shooterSubsystem);
    }
}
