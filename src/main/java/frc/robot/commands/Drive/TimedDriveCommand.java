package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class TimedDriveCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final double m_xVelocity;
    private final double m_yVelocity;
    private final double m_rotationVelocity;

    private final double m_timeInSeconds;
    private Timer m_timer;

    /**
     * This command is meant to drive with a certain velocity for a certain amount
     * of time.
     * This is mostly meant for a simple autonomous command, but can later be
     * reused.
     */
    public TimedDriveCommand(DriveSubsystem driveSubsystem, double xVelocity, double yVelocity,
            double rotationVelocity, double timeInSeconds) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_xVelocity = xVelocity;
        this.m_yVelocity = yVelocity;
        this.m_rotationVelocity = rotationVelocity;

        this.m_timeInSeconds = timeInSeconds;
        this.m_timer = new Timer();

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.m_timer.start();
        this.m_driveSubsystem.setModules(this.m_xVelocity, this.m_yVelocity, this.m_rotationVelocity);
    }

    @Override
    public boolean isFinished() {
        return this.m_timer.hasElapsed(this.m_timeInSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_timer.stop();
        this.m_timer.reset();
        this.m_driveSubsystem.setModules(0, 0, 0);
    }
}
