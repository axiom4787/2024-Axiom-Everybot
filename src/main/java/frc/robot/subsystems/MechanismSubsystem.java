package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MechState;

public class MechanismSubsystem extends SubsystemBase {
    private final CANSparkMax m_bottomShooter, m_topShooter, m_roller;
    private MechState m_shooterState, m_rollerState, m_climberState;

    public MechanismSubsystem()
    {
        m_bottomShooter = new CANSparkMax(DriveConstants.kBottomShooterCanId, MotorType.kBrushless);
        m_topShooter = new CANSparkMax(DriveConstants.kTopShooterCanId, MotorType.kBrushless);
        m_roller = new CANSparkMax(DriveConstants.kGrabberCanId, MotorType.kBrushed);
        m_topShooter.setSmartCurrentLimit(40);
        m_bottomShooter.setSmartCurrentLimit(40);
        m_roller.setSmartCurrentLimit(25);
//        m_leftClimber = new CANSparkMax(DriveConstants.kLeftClimberCanId, MotorType.kBrushed);
//        m_rightClimber = new CANSparkMax(DriveConstants.kRightClimberCanId, MotorType.kBrushed);
//        m_leftClimber.setSmartCurrentLimit(40);
//        m_rightClimber.setSmartCurrentLimit(40);
    }   

    public void setShooterState(MechState state)
    {
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

    public Command mechCommand()
    {
        return this.run(() -> {
            switch (m_shooterState)
            {
                case mNegative:
                    m_topShooter.set(-DriveConstants.kShooterSpeed);
                    m_bottomShooter.set(-DriveConstants.kShooterSpeed);
                    break;
                case mPositive:
                    m_topShooter.set(DriveConstants.kShooterSpeed);
                    m_bottomShooter.set(DriveConstants.kShooterSpeed);
                    break;
                case mReset:
                    m_topShooter.set(0);
                    m_bottomShooter.set(0);
            }
            switch (m_rollerState)
            {
                case mNegative:
                    m_roller.set(-DriveConstants.kRollerSpeed);
                    break;
                case mPositive:
                    m_roller.set(DriveConstants.kRollerSpeed);
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
        });
    }
}
