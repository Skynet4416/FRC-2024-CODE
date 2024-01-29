// https://github.com/FRCTeam2910/2022CompetitionRobot/blob/master/src/main/java/org/frcteam2910/c2022/subsystems/ClimberSubsystem.java
package frc.robot.subsystems.Climber;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;
    private boolean m_isOpen;

    public ClimberSubsystem() {
        this.m_leftMotor = new CANSparkMax(Climber.Motors.kLeftHookMotorID, MotorType.kBrushless);
        this.m_rightMotor = new CANSparkMax(Climber.Motors.kRightHookMotorID, MotorType.kBrushless);

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climber");
        // shuffleboardTab.addNumber("Height", () -> {return getCurrentHeight();});
        shuffleboardTab.addBoolean("is open?", () -> {return m_isOpen;});

        setNeutralMode(IdleMode.kBrake);

        this.m_isOpen = false;
    }

    public void configMotors() {
        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftMotor.enableVoltageCompensation(12);
        m_rightMotor.enableVoltageCompensation(12);
        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);

    }

    public void setNeutralMode(IdleMode idleMode) 
    {
        m_leftMotor.setIdleMode(idleMode);
        m_rightMotor.setIdleMode(idleMode);
    }


    public boolean isOpen() 
    {
        return m_isOpen;
    }

    public void extendTelescope()
    {
        if(!m_isOpen)
        {
            m_leftMotor.set(Climber.Stats.kExtendSpeed);
            m_rightMotor.set(Climber.Stats.kExtendSpeed);
            try 
            {
                Thread.sleep(Climber.Stats.kExtendTimeMS);
            } 
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            m_leftMotor.stopMotor();
            m_rightMotor.stopMotor();
        }
        
        m_isOpen = true;
    }

     public void retractTelescope()
    {
        if(m_isOpen)
        {
            m_leftMotor.set(-Climber.Stats.kExtendSpeed);
            m_rightMotor.set(-Climber.Stats.kExtendSpeed);
            try 
            {
                Thread.sleep(Climber.Stats.kRetractTimeMS);
            } 
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            m_leftMotor.stopMotor();
            m_rightMotor.stopMotor();
        }
        m_isOpen = false;
    }
}