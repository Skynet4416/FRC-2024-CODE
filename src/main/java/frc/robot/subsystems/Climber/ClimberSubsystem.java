// https://github.com/FRCTeam2910/2022CompetitionRobot/blob/master/src/main/java/org/frcteam2910/c2022/subsystems/ClimberSubsystem.java
package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AllRobot;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;
    private boolean m_isOpen;

    public ClimberSubsystem() {
        this.m_leftMotor = new CANSparkMax(Climber.Motors.kLeftHookMotorID, MotorType.kBrushless);
        this.m_rightMotor = new CANSparkMax(Climber.Motors.kRightHookMotorID, MotorType.kBrushless);

        m_leftMotor.getPIDController().setP(Climber.PID.kP);
        m_leftMotor.getPIDController().setI(Climber.PID.kI);
        m_leftMotor.getPIDController().setD(Climber.PID.kD);

        m_rightMotor.getPIDController().setP(Climber.PID.kP);
        m_rightMotor.getPIDController().setI(Climber.PID.kI);
        m_rightMotor.getPIDController().setD(Climber.PID.kD);

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climber");
        // shuffleboardTab.addNumber("Height", () -> {return getCurrentHeight();});
        shuffleboardTab.addBoolean("is open?", () -> {
            return m_isOpen;
        });

        setNeutralMode(IdleMode.kBrake);

        this.m_isOpen = false;
    }

    public void configMotors() {
        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftMotor.enableVoltageCompensation(12);
        m_rightMotor.enableVoltageCompensation(12);
        // m_leftMotor.setInverted(false);
        // m_rightMotor.setInverted(true);

    }

    public void setNeutralMode(IdleMode idleMode) {
        m_leftMotor.setIdleMode(idleMode);
        m_rightMotor.setIdleMode(idleMode);
    }

    public boolean isOpen() {
        return m_isOpen;
    }

    public void extendTelescope() {
        m_leftMotor.getPIDController().setReference(Climber.Stats.kExtensionTurnsInRounds, ControlType.kPosition);
        m_rightMotor.getPIDController().setReference(-Climber.Stats.kExtensionTurnsInRounds, ControlType.kPosition);
        this.m_isOpen = true;
    }

    public void retractTelescope() {
        m_leftMotor.getPIDController().setReference(Climber.Stats.kRetractInRounds, ControlType.kPosition);
        m_rightMotor.getPIDController().setReference(Climber.Stats.kRetractInRounds, ControlType.kPosition);
        this.m_isOpen = false;
    }

    public boolean isExtended() {
        return Math.abs(m_leftMotor.getEncoder().getPosition()
                - Climber.Stats.kExtensionTurnsInRounds) < Climber.Stats.kThreashold;
    }

    public boolean isRetracted() {
        return Math.abs(m_leftMotor.getEncoder().getPosition()
                - Climber.Stats.kRetractInRounds) < Climber.Stats.kThreashold;
    }

    public void init() {
        m_leftMotor.setSmartCurrentLimit(AllRobot.kAllMotorsLimitInAmpr);
        m_rightMotor.setSmartCurrentLimit(AllRobot.kAllMotorsLimitInAmpr);
    }

    public void setVoltage(double voltage) {
        m_leftMotor.setVoltage(voltage);
        m_rightMotor.setVoltage(voltage);

    }
}