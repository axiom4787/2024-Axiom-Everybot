// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoMode;

/** Add your docs here. */
public class Robot extends TimedRobot {
    private Command m_teleopCommand, m_autoCommand;
    private RobotContainer m_container;
    private final SendableChooser<AutoMode> m_chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("do nothing", AutoMode.amNothing);
        m_chooser.addOption("launch note and drive", AutoMode.amLaunchDrive);
        m_chooser.addOption("launch", AutoMode.amLaunch);
        m_chooser.addOption("drive", AutoMode.amDrive);
        SmartDashboard.putData("Auto Choices", m_chooser);
        m_container = new RobotContainer(m_chooser);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        m_teleopCommand = m_container.getTeleopCommand();
        if (m_teleopCommand != null)
            m_teleopCommand.schedule();
    }

    @Override
    public void autonomousInit() {
        m_autoCommand = m_container.getAutoCommand();
        if (m_autoCommand != null)
            m_autoCommand.schedule();
    }
}
