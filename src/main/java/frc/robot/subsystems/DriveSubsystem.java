package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkBase m_leftRear, m_leftFront, m_rightRear, m_rightFront;
    private final DifferentialDrive m_driveTrain;
    public DriveSubsystem() {
        m_leftRear = new CANSparkMax(0, MotorType.kBrushed);
        m_leftFront = new CANSparkMax(0, MotorType.kBrushed);
        m_rightRear = new CANSparkMax(0, MotorType.kBrushed);
        m_rightFront = new CANSparkMax(0, MotorType.kBrushed);
        m_leftRear.setSmartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT_A);
        m_leftFront.setSmartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT_A);
        m_rightRear.setSmartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT_A);
        m_rightFront.setSmartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT_A);
        m_leftRear.follow(m_leftFront);
        m_rightRear.follow(m_rightFront);
        m_leftFront.setInverted(true);
        m_rightFront.setInverted(false);
        m_driveTrain = new DifferentialDrive(m_leftFront, m_rightFront);
    }

    public void startCoast() {
        m_leftRear.setIdleMode(IdleMode.kCoast);
        m_leftFront.setIdleMode(IdleMode.kCoast);
        m_rightRear.setIdleMode(IdleMode.kCoast);
        m_rightFront.setIdleMode(IdleMode.kCoast);
    }

    public void drive(double throttle, double turn) {
        m_driveTrain.arcadeDrive(throttle, turn);
    }
}