package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MechState;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ShooterConstants;

public class ClimberSubsystem extends SubsystemBase
{
    private final CANSparkMax m_leftClimber, m_rightClimber;
    private final BlinkinLights m_lights;
    public ClimberSubsystem(BlinkinLights lights)
    {
        m_lights = lights;
        m_leftClimber = new CANSparkMax(ClimberConstants.kLeftCanId, MotorType.kBrushless);
        m_rightClimber = new CANSparkMax(ClimberConstants.kRightCanId, MotorType.kBrushless);
        m_leftClimber.setSmartCurrentLimit(30);
        m_rightClimber.setSmartCurrentLimit(30);
        m_leftClimber.setIdleMode(IdleMode.kBrake);
        m_rightClimber.setIdleMode(IdleMode.kBrake);
    }
    public void setClimberSpeed(double speed)
    {
        if (speed != 0)
            m_lights.set(BlinkinConstants.kClimbing);
        else
            m_lights.set(BlinkinConstants.kEmpty);
        System.out.println(speed);
        m_leftClimber.set(speed);
        m_rightClimber.set(speed);
    }
}