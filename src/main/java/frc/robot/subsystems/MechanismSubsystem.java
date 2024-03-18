package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.MechState;

public class MechanismSubsystem extends SubsystemBase {
    private final CANSparkMax m_frontShooter, m_backShooter;
    private final CANSparkMax m_indexer;
    private final CANSparkMax m_roller;
    private MechState m_shooterState, m_rollerState;
    private final Timer m_indexerStartTimer = new Timer();

    public MechanismSubsystem()
    {
        m_frontShooter = new CANSparkMax(ShooterConstants.kFrontCanId, MotorType.kBrushless);
        m_backShooter = new CANSparkMax(ShooterConstants.kBackCanId, MotorType.kBrushless);
        m_indexer = new CANSparkMax(ShooterConstants.kIndexerCanId, MotorType.kBrushed);
        m_roller = new CANSparkMax(RollerConstants.kCanId, MotorType.kBrushless);
        m_frontShooter.setSmartCurrentLimit(30);
        m_backShooter.setSmartCurrentLimit(30);
        m_indexer.setSmartCurrentLimit(20);
        m_roller.setSmartCurrentLimit(30);
        m_frontShooter.setIdleMode(IdleMode.kCoast);
        m_backShooter.setIdleMode(IdleMode.kCoast);
        m_indexer.setIdleMode(IdleMode.kCoast);
        m_roller.setIdleMode(IdleMode.kCoast);
        m_shooterState = m_rollerState = MechState.mReset;
    }   

    @Override
    public void periodic()
    {
        switch (m_shooterState)
        {
            case mNegative:
                m_frontShooter.set(ShooterConstants.kShooterIntakeSpeed);
                m_backShooter.set(ShooterConstants.kShooterIntakeSpeed);
                m_indexer.set(ShooterConstants.kIndexerIntakeSpeed);
                break;
            case mPositive:
                m_frontShooter.set(ShooterConstants.kShooterLaunchSpeed);
                m_backShooter.set(ShooterConstants.kShooterLaunchSpeed);
                if (m_indexerStartTimer.get() > ShooterConstants.kIndexerDelay)
                    m_indexer.set(ShooterConstants.kIndexerLaunchSpeed);
                else
                    m_indexer.set(0);
                break;
            case mReset:
                m_frontShooter.set(0);
                m_backShooter.set(0);
                m_indexer.set(0);
        }
        switch (m_rollerState)
        {
            case mNegative:
                m_roller.set(RollerConstants.kRollerSpeed);
                break;
            case mPositive:
                m_roller.set(-RollerConstants.kRollerSpeed);
                break;
            case mReset:
                m_roller.set(0);
        }
    }

    public void setShooterState(MechState state)
    {
        if (state == MechState.mPositive && m_shooterState == MechState.mReset)
        {
            m_indexerStartTimer.reset();
            m_indexerStartTimer.start();
        }
        m_shooterState = state;
    }

    public void setRollerState(MechState state)
    {
        m_rollerState = state;
    }

    public MechState getShooterState()
    {
        return m_shooterState;
    }

    public MechState getRollerState()
    {
        return m_rollerState;
    }

}