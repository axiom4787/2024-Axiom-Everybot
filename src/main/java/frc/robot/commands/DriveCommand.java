// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LaunchFeedSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.XboxController;

public class DriveCommand extends Command {
  private final DriveSubsystem m_driveSystem;
  private final LaunchFeedSubsystem m_launchFeedSystem;
  private final ClimberSubsystem m_climberSystem;
  private XboxController m_controller;

  public DriveCommand(DriveSubsystem driveSystem, LaunchFeedSubsystem launchFeedSystem, ClimberSubsystem climberSystem) {
    m_driveSystem = driveSystem;
    m_launchFeedSystem = launchFeedSystem;
    m_climberSystem = climberSystem;
    addRequirements(driveSystem);
    addRequirements(launchFeedSystem);
    addRequirements(climberSystem);
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
      m_launchFeedSystem.launchControl(true);
    else if (m_controller.getRightBumperReleased())
      m_launchFeedSystem.launchControl(false);

    if (m_controller.getLeftBumperPressed())
      m_launchFeedSystem.feedControl(true);
    else if (m_controller.getLeftBumperReleased())
      m_launchFeedSystem.feedControl(false);
    
    if (m_controller.getAButtonPressed())
      m_launchFeedSystem.ampControl(true);
    else if (m_controller.getAButtonReleased())
      m_launchFeedSystem.ampControl(false);
    
    if (m_controller.getBButtonPressed())
      m_launchFeedSystem.intakeControl(true);
    else if (m_controller.getBButtonReleased())
      m_launchFeedSystem.intakeControl(false);
    
    /* Climber commands
     * X: extend arms to hook chain
     * Y: retract arms to raise robot
     * These button bindings can be changed later if the driver prefers
     */
    if (m_controller.getXButtonPressed()) {
      System.out.println("Extending arm to hook onto chain...");
      m_climberSystem.extendArms();
    } else if (m_controller.getXButtonReleased()) { // motor speed is 0 if no button pressed
      System.out.println("Reseting climber motor...");
      m_climberSystem.resetMotor();
    }
    if (m_controller.getYButtonPressed()) {
      System.out.println("Retracting arm to raise robot onto chain...");
      m_climberSystem.retractArms();
    } else if (m_controller.getYButtonReleased()) { // motor speed is 0 if no button pressed
      System.out.println("Reseting climber motor...");
      m_climberSystem.resetMotor();
    }
    
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
