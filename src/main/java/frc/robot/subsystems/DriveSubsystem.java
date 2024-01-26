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
        m_leftRear = new CANSparkMax(DriveConstants.LEFT_REAR_MOTOR_ID, MotorType.kBrushed);
        m_leftFront = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID, MotorType.kBrushed);
        m_rightRear = new CANSparkMax(DriveConstants.RIGHT_REAR_MOTOR_ID, MotorType.kBrushed);
        m_rightFront = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, MotorType.kBrushed);
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

    // sets all wheels to specified idle mode
    public void setIdleMode(IdleMode mode) {
        m_leftRear.setIdleMode(mode);
        m_leftFront.setIdleMode(mode);
        m_rightRear.setIdleMode(mode);
        m_rightFront.setIdleMode(mode);
    }

    // drives robot with specified throttle/turn values
    public void drive(double throttle, double turn) {
        m_driveTrain.arcadeDrive(throttle, turn);
    }
}