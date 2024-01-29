// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoMode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LaunchFeedSubsystem;

public class AutoCommand extends Command {
  /** Creates a new AutoCommand. */
  private final DriveSubsystem m_driveSystem;
  private final LaunchFeedSubsystem m_mechSystem;
  private final SendableChooser<AutoMode> m_chooser = new SendableChooser<>();
  private AutoMode m_autoSelected;
  private double AUTO_LAUNCH_DELAY_S, AUTO_DRIVE_DELAY_S, 
  AUTO_DRIVE_TIME_S, AUTO_DRIVE_SPEED, AUTO_LAUNCHER_SPEED, autonomousStartTime;
  public AutoCommand(DriveSubsystem driveSystem, LaunchFeedSubsystem mechSystem) {
    m_driveSystem = driveSystem;
    m_mechSystem = mechSystem;
    m_chooser.setDefaultOption("do nothing", AutoMode.amNothing);
    m_chooser.addOption("launch note and drive", AutoMode.amLaunchDrive);
    m_chooser.addOption("launch", AutoMode.amLaunch);
    m_chooser.addOption("drive", AutoMode.amDrive);
    SmartDashboard.putData("Auto Choices", m_chooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autoSelected = m_chooser.getSelected();

    AUTO_LAUNCH_DELAY_S = 2;
    AUTO_DRIVE_DELAY_S = 3;

    AUTO_DRIVE_TIME_S = 2.0;
    AUTO_DRIVE_SPEED = -0.5;
    AUTO_LAUNCHER_SPEED = 1;
    if(m_autoSelected == AutoMode.amLaunch)
    {
      AUTO_DRIVE_SPEED = 0;
      AUTO_LAUNCHER_SPEED = 1;
    }
    else if(m_autoSelected == AutoMode.amDrive)
    {
      AUTO_LAUNCHER_SPEED = 0;
      AUTO_DRIVE_SPEED = -0.5;
    }
    else if(m_autoSelected == AutoMode.amLaunchDrive)
    {
      AUTO_DRIVE_SPEED = -0.5;
      AUTO_LAUNCHER_SPEED = 1;
    }
    else if(m_autoSelected == AutoMode.amNothing)
    {
      AUTO_DRIVE_SPEED = 0;
      AUTO_LAUNCHER_SPEED = 0;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    if (timeElapsed < AUTO_LAUNCH_DELAY_S)
    {
      m_mechSystem.launchWheelSet(AUTO_LAUNCHER_SPEED);
//      m_driveSystem.drive(0, 0);
    }
    else if (timeElapsed < AUTO_DRIVE_DELAY_S)
    {
      m_mechSystem.feedWheelSet(AUTO_LAUNCHER_SPEED);
//      m_driveSystem.drive(0, 0);
    }
    else if (timeElapsed < AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S)
    {
      m_mechSystem.feedWheelSet(0);
      m_mechSystem.launchWheelSet(0);
//      m_driveSystem.drive(AUTO_DRIVE_SPEED, 0);
    }
    else
    {
//      m_driveSystem.drive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
