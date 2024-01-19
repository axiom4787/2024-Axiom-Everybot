// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.FeederConstants;
import static frc.robot.Constants.LauncherConstants;

/** Add your docs here. */
public class LaunchFeedSubsystem extends SubsystemBase {
    private final CANSparkBase m_launchWheel;
    private final CANSparkBase m_feedWheel; 
    public LaunchFeedSubsystem()
    {
        m_launchWheel = new CANSparkMax(6, MotorType.kBrushed);
        m_feedWheel = new CANSparkMax(5, MotorType.kBrushed);  
        m_feedWheel.setInverted(true);
        m_launchWheel.setInverted(true);
        m_feedWheel.setSmartCurrentLimit(FeederConstants.FEEDER_CURRENT_LIMIT_A);
        m_launchWheel.setSmartCurrentLimit(LauncherConstants.LAUNCHER_CURRENT_LIMIT_A);        
    }

    // auto control commands

    // sets launch wheel speed
    public void launchWheelSet(double speed)
    {
        m_launchWheel.set(speed);
    }

    // sets feed wheel speed
    public void feedWheelSet(double speed)
    {
        m_feedWheel.set(speed);
    }

    // teleop control commands

    // starts/stops spinning up launch wheel
    public void launchControl(boolean isActive)
    {
        if (isActive) {
            m_launchWheel.set(LauncherConstants.LAUNCHER_SPEED);
        }
        else {
            m_launchWheel.set(0);
        }
    }

    // starts/stops spinning up feed wheel
    public void feedControl(boolean isActive)
    {
        if (isActive) {
            m_feedWheel.set(FeederConstants.FEEDER_OUT_SPEED);
        }
        else {
            m_feedWheel.set(0);
        }
    }

    // starts/stops spinning both wheels to intake note
    public void intakeControl(boolean isActive)
    {
        if (isActive) {
            m_feedWheel.set(FeederConstants.FEEDER_IN_SPEED);
            m_launchWheel.set(-LauncherConstants.LAUNCHER_SPEED);
        }
        else {
            m_feedWheel.set(0);
            m_launchWheel.set(0);
        }
    }

    // starts/stops spinning both wheels to shoot note into amp
    public void ampControl(boolean isActive)
    {
        if (isActive) {
            m_feedWheel.set(FeederConstants.FEEDER_AMP_SPEED);
            m_launchWheel.set(LauncherConstants.LAUNCHER_AMP_SPEED);
        }
        else {
            m_feedWheel.set(0);
            m_launchWheel.set(0);
        }
    }
}
