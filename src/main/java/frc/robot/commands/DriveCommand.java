// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.StadiaController;

public class DriveCommand extends Command {
  private final DriveSubsystem m_driveSystem;
  private XboxController m_xboxController;
  private StadiaController m_stadiaController;

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
    if (Constants.xbox)
      m_xboxController = new XboxController(0);
    else
      m_stadiaController = new StadiaController(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.xbox) {
      if (m_xboxController.getBButtonPressed())
        isFieldRelative = !isFieldRelative;
      if (m_xboxController.getYButtonPressed())
        m_driveSystem.toggleCam();
      if (m_xboxController.getAButtonPressed() && isFieldRelative)
        m_driveSystem.zeroHeading();
      if (m_xboxController.getXButtonPressed())
        isDefending = !isDefending;
      if (isDefending) {
        m_driveSystem.setX();
        return;
      }
      double xSpeed, ySpeed, rot;
      if (m_xboxController.getPOV() != -1) {
        double POV = Math.toRadians(-m_xboxController.getPOV());
        xSpeed = OIConstants.kDriveAdjustSpeed*Math.cos(POV);
        ySpeed = OIConstants.kDriveAdjustSpeed*Math.sin(POV); 
      }
      else {
        xSpeed = -MathUtil.applyDeadband(m_xboxController.getLeftY()/3, OIConstants.kDriveDeadband);
        ySpeed = -MathUtil.applyDeadband(m_xboxController.getLeftX()/3, OIConstants.kDriveDeadband);
      }
      if (m_xboxController.getBackButton())
        rot = -OIConstants.kTurnAdjustSpeed;
      else if (m_xboxController.getStartButton())
        rot = OIConstants.kTurnAdjustSpeed;
      else
        rot = -MathUtil.applyDeadband(m_xboxController.getRightX()/2, OIConstants.kDriveDeadband);
      m_driveSystem.drive(xSpeed, ySpeed, rot, isFieldRelative, true, isTrackingObject, isAvoidingObject, isBalancing);
    }
    else {
      if (m_stadiaController.getBButtonPressed())
        isFieldRelative = !isFieldRelative;
      if (m_stadiaController.getYButtonPressed())
        m_driveSystem.toggleCam();
      if (m_stadiaController.getAButtonPressed() && isFieldRelative)
        m_driveSystem.zeroHeading();
      if (m_stadiaController.getXButtonPressed())
        isDefending = !isDefending;
      if (isDefending) {
        m_driveSystem.setX();
        return;
      }
      double xSpeed, ySpeed, rot;
      if (m_stadiaController.getPOV() != -1) {
        double POV = Math.toRadians(-m_stadiaController.getPOV());
        xSpeed = OIConstants.kDriveAdjustSpeed*Math.cos(POV);
        ySpeed = OIConstants.kDriveAdjustSpeed*Math.sin(POV); 
      }
      else {
        xSpeed = -MathUtil.applyDeadband(m_stadiaController.getLeftY()/3, OIConstants.kDriveDeadband);
        ySpeed = -MathUtil.applyDeadband(m_stadiaController.getLeftX()/3, OIConstants.kDriveDeadband);
      }
      if (m_stadiaController.getEllipsesButton())
        rot = -OIConstants.kTurnAdjustSpeed;
      else if (m_stadiaController.getHamburgerButton())
        rot = OIConstants.kTurnAdjustSpeed;
      else
        rot = -MathUtil.applyDeadband(m_stadiaController.getRightX()/2, OIConstants.kDriveDeadband);
      m_driveSystem.drive(xSpeed, ySpeed, rot, isFieldRelative, true, isTrackingObject, isAvoidingObject, isBalancing);
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
