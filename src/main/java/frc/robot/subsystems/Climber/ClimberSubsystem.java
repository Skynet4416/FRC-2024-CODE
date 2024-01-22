// https://github.com/FRCTeam2910/2022CompetitionRobot/blob/master/src/main/java/org/frcteam2910/c2022/subsystems/ClimberSubsystem.java
package frc.robot.subsystems.Climber;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    public ClimberSubsystem() {
        m_leftMotor = new CANSparkMax(Climber.Motors.kLeftHookMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(Climber.Motors.kRightHookMotorID, MotorType.kBrushless);


        setNeutralMode(IdleMode.kBrake);
    }

    public void configMotors() {
        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftMotor.enableVoltageCompensation(12);
        m_rightMotor.enableVoltageCompensation(12);
        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);

    }

    


    public void setNeutralMode(IdleMode idleMode) {
        m_leftMotor.setIdleMode(idleMode);
        m_rightMotor.setIdleMode(idleMode);
    }

}   