// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Robot extends TimedRobot {
    private Command m_teleopCommand, m_autoCommand;
    private RobotContainer m_container;
    @Override
    public void robotInit() {
        m_container = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
        CommandScheduler.getInstance().run();
        if (m_autoCommand != null && m_autoCommand.isFinished()) {
            m_autoCommand.cancel();
            System.out.println("Auto command detected as finished, cancelling now.");
            m_autoCommand = null;
            m_container.getZeroCommand().schedule();
            System.out.println("Scheduled zero-chassis command");
        }
    }

    @Override
    public void teleopInit() {
        if (m_autoCommand != null && !m_autoCommand.isFinished()) {
            m_autoCommand.cancel();
        }

        m_teleopCommand = m_container.getTeleopCommand();
        if (m_teleopCommand != null)
            m_teleopCommand.schedule();
    }

    @Override
    public void autonomousInit() {
        if (m_teleopCommand != null && !m_teleopCommand.isFinished()) {
            m_teleopCommand.cancel();
        }

        m_autoCommand = m_container.getAutoCommand();
        if (m_autoCommand != null)
            m_autoCommand.schedule();
    }
}
