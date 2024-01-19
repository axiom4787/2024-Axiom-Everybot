// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.XboxController;

public class DriveCommand extends Command {
  private final DriveSubsystem m_driveSystem;
  private final MechanismSubsystem m_mechSystem;
  private XboxController m_controller;

  public DriveCommand(DriveSubsystem driveSystem, MechanismSubsystem mechSystem) {
    m_driveSystem = driveSystem;
    m_mechSystem = mechSystem;
    addRequirements(driveSystem);
    addRequirements(mechSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSystem.setIdleMode(IdleMode.kCoast);
    m_controller = new XboxController(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRightBumperPressed())
      m_mechSystem.launchControl(true);
    else if (m_controller.getRightBumperReleased())
      m_mechSystem.launchControl(false);

    if (m_controller.getLeftBumperPressed())
      m_mechSystem.feedControl(true);
    else if (m_controller.getLeftBumperReleased())
      m_mechSystem.feedControl(false);
    
    if (m_controller.getAButtonPressed())
      m_mechSystem.ampControl(true);
    else if (m_controller.getAButtonReleased())
      m_mechSystem.ampControl(false);
    
    if (m_controller.getBButtonPressed())
      m_mechSystem.intakeControl(true);
    else if (m_controller.getBButtonReleased())
      m_mechSystem.intakeControl(false);
    
    m_driveSystem.drive(m_controller.getLeftY(), m_controller.getRightX());
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
