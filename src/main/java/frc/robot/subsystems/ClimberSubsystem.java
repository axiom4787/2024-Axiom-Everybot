package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase
{
    private final CANSparkMax m_leftClimber, m_rightClimber;
    public ClimberSubsystem()
    {
        m_leftClimber = new CANSparkMax(ClimberConstants.kLeftCanId, MotorType.kBrushless);
        m_rightClimber = new CANSparkMax(ClimberConstants.kRightCanId, MotorType.kBrushless);
        m_leftClimber.setSmartCurrentLimit(30);
        m_rightClimber.setSmartCurrentLimit(30);
        m_leftClimber.setIdleMode(IdleMode.kBrake);
        m_rightClimber.setIdleMode(IdleMode.kBrake);
    }

    public void setLeftClimber(double speed)
    {
        m_leftClimber.set(
            MathUtil.clamp(speed, -1, 1));
    }

    public void setRightClimber(double speed)
    {
        m_rightClimber.set(
            MathUtil.clamp(speed, -1, 1));
    }
}