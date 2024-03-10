// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// Climber subsystem
// to climb onto the chain at the end of the match
public class ClimberSubsystem extends SubsystemBase {
    /*
     * Assuming a single motor for now (will change later if needed)
     * (As in, something like this https://www.andymark.com/products/climber-in-a-box)
     * As com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX cannot import on my computer right now,
     * the motor object is a CANSparkMax. can of course change if needed.
     */
    private final CANSparkBase m_motor;

    public ClimberSubsystem() {
        m_motor = new CANSparkMax(DriveConstants.kClimberMotorID, MotorType.kBrushed);
    }

    public void extendArms() { // hook onto chain
        m_motor.set(DriveConstants.kClimberMotorSpeed);
    }

    public void retractArms() { // pull up onto chain
        m_motor.set(-DriveConstants.kClimberMotorSpeed);
    }

    public void resetMotor() { // reset motor speed after done
        m_motor.set(0);
    }
}
