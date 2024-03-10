// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class DriveCommand extends Command {
  private final DriveSubsystem m_driveSystem;
  private XboxController m_controller;

  public DriveCommand(DriveSubsystem driveSystem) {
    m_driveSystem = driveSystem;
    addRequirements(driveSystem);
  }

  private boolean isFieldRelative = true;
  private boolean isBalancing = false;
  private boolean isTrackingObject = false;
  private boolean isAvoidingObject = false;
  private boolean isDefending = false;
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller = new XboxController(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getBButtonPressed())
      isFieldRelative = !isFieldRelative;
    if (m_controller.getYButtonPressed())
      m_driveSystem.toggleCam();
    if (m_controller.getAButtonPressed() && isFieldRelative)
      m_driveSystem.zeroHeading();
    if (m_controller.getXButtonPressed())
      isDefending = !isDefending;
    if (isDefending) {
      m_driveSystem.setX();
      return;
    }
    double xSpeed, ySpeed, rot;
    if (m_controller.getPOV() != -1) {
      double POV = Math.toRadians(-m_controller.getPOV()+90);
      xSpeed = OIConstants.kAdjustSpeed*Math.cos(POV);
      ySpeed = OIConstants.kAdjustSpeed*Math.sin(POV); 
    }
    else {
      xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY()/3, OIConstants.kDriveDeadband);
      ySpeed = -MathUtil.applyDeadband(m_controller.getLeftX()/3, OIConstants.kDriveDeadband);
    }
    if (m_controller.getBackButton())
      rot = -OIConstants.kAdjustSpeed;
    else if (m_controller.getStartButton())
      rot = OIConstants.kAdjustSpeed;
    else
      rot = -MathUtil.applyDeadband(m_controller.getRightX()/2, OIConstants.kDriveDeadband);
    m_driveSystem.drive(xSpeed, ySpeed, rot, isFieldRelative, true, isTrackingObject, isAvoidingObject, isBalancing);
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
