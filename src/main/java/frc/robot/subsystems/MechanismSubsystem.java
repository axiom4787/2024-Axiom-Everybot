package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismSubsystem extends SubsystemBase {
    private final CANSparkMax m_bottomShooter;
    private final CANSparkMax m_topShooter;
//    private final CANSparkMax m_grabber;
//    private final CANSparkMax m_leftClimber;
//    private final CANSparkMax m_rightClimber;

    public MechanismSubsystem()
    {
        m_bottomShooter = new CANSparkMax(DriveConstants.kBottomShooterCanId, MotorType.kBrushed);
        m_topShooter = new CANSparkMax(DriveConstants.kTopShooterCanId, MotorType.kBrushed);
//        m_grabber = new CANSparkMax(DriveConstants.kGrabberCanId, MotorType.kBrushed);
//        m_leftClimber = new CANSparkMax(DriveConstants.kLeftClimberCanId, MotorType.kBrushed);
//        m_rightClimber = new CANSparkMax(DriveConstants.kRightClimberCanId, MotorType.kBrushed);
    }   

    public void intakeNote()
    {
        m_bottomShooter.set(-DriveConstants.kShooterSpeed);
        m_topShooter.set(-DriveConstants.kShooterSpeed);
    }

    public void shootNote()
    {
        m_bottomShooter.set(DriveConstants.kShooterSpeed);
        m_topShooter.set(DriveConstants.kShooterSpeed);
    }

    public void resetShooter()
    {
        m_bottomShooter.set(0);
        m_topShooter.set(0);
    }

    // public void grabNote()
    // {
    //     m_grabber.set(DriveConstants.kGrabberSpeed);
    // }

    // public void dropNote()
    // {
    //     m_grabber.set(-DriveConstants.kGrabberSpeed);
    // }

    // public void resetGrabber()
    // {
    //     m_grabber.set(0);
    // }

    // public void extendArms() {
    //     m_leftClimber.set(DriveConstants.kClimberMotorSpeed);
    //     m_rightClimber.set(DriveConstants.kClimberMotorSpeed);
    // }

    // public void retractArms() {
    //     m_leftClimber.set(-DriveConstants.kClimberMotorSpeed);
    //     m_rightClimber.set(-DriveConstants.kClimberMotorSpeed);
    // }

    // public void resetArmMotors() {
    //     m_leftClimber.set(0);
    //     m_rightClimber.set(0);
    // }
}
