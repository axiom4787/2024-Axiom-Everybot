package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MechState;

public class MechanismSubsystem extends SubsystemBase {
    private final CANSparkMax m_frontShooter, m_backShooter;
    private final CANSparkMax m_lowerShooter;
    private final CANSparkMax m_roller;
    private MechState m_shooterState, m_rollerState, m_climberState;

    public MechanismSubsystem()
    {
        m_frontShooter = new CANSparkMax(LauncherConstants.kFrontCanId, MotorType.kBrushless);
        m_backShooter = new CANSparkMax(LauncherConstants.kBackCanId, MotorType.kBrushless);
        m_lowerShooter = new CANSparkMax(LauncherConstants.kLowerCanId, MotorType.kBrushed);
        m_roller = new CANSparkMax(RollerConstants.kCanId, MotorType.kBrushed);
        m_frontShooter.setSmartCurrentLimit(40);
        m_backShooter.setSmartCurrentLimit(40);
        m_lowerShooter.setSmartCurrentLimit(25);
        m_roller.setSmartCurrentLimit(25);
        m_shooterState = m_climberState = m_rollerState = MechState.mReset;
//        m_leftClimber = new CANSparkMax(DriveConstants.kLeftClimberCanId, MotorType.kBrushed);
//        m_rightClimber = new CANSparkMax(DriveConstants.kRightClimberCanId, MotorType.kBrushed);
//        m_leftClimber.setSmartCurrentLimit(40);
//        m_rightClimber.setSmartCurrentLimit(40);
    }   

    @Override
    public void periodic()
    {
        switch (m_shooterState)
        {
            case mNegative:
                m_frontShooter.set(LauncherConstants.kUpperIntakeSpeed);
                m_backShooter.set(LauncherConstants.kUpperIntakeSpeed);
                m_lowerShooter.set(LauncherConstants.kLowerIntakeSpeed);
                break;
            case mPositive:
                m_frontShooter.set(LauncherConstants.kUpperLauncherSpeed);
                m_backShooter.set(LauncherConstants.kUpperLauncherSpeed);
                m_lowerShooter.set(LauncherConstants.kLowerLauncherSpeed);
                break;
            case mReset:
                m_frontShooter.set(0);
                m_backShooter.set(0);
                m_lowerShooter.set(0);
        }
        switch (m_rollerState)
        {
            case mNegative:
                m_roller.set(-RollerConstants.kRollerSpeed);
                break;
            case mPositive:
                m_roller.set(RollerConstants.kRollerSpeed);
                break;
            case mReset:
                m_roller.set(0);
        }
        // switch (m_climberState)
        // {
        //     case mNegative:
        //         m_leftClimber.set(-DriveConstants.kClimberSpeed);
        //         m_rightClimber.set(-DriveConstants.kClimberSpeed);
        //         break;
        //     case mPositive:
        //         m_leftClimber.set(DriveConstants.kClimberSpeed);
        //         m_rightClimber.set(DriveConstants.kClimberSpeed);
        //         break;
        //     case mReset:
        //         m_leftClimber.set(0);
        //         m_rightClimber.set(0);
        // }
    }

    public void setShooterState(MechState state)
    {
        System.out.println("hi");
        m_shooterState = state;
    }

    public void setRollerState(MechState state)
    {
        m_rollerState = state;
    }

    public void setClimberState(MechState state)
    {
        m_climberState = state;
    }

    public MechState getShooterState()
    {
        return m_shooterState;
    }

    public MechState getRollerState()
    {
        return m_rollerState;
    }

    public MechState getClimberState()
    {
        return m_climberState;
    }

}
