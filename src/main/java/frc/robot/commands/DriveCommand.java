// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

public class DriveCommand extends Command {
  private final DriveSubsystem m_driveSystem;
  private XboxController m_controller;
  private Joystick m_joystick;

  public DriveCommand(DriveSubsystem driveSystem) {
    m_driveSystem = driveSystem;
    addRequirements(driveSystem);
  }

  private boolean isFieldRelative = true;
  private boolean isBalancing = false;
  private boolean isTrackingObject = false;
  private boolean isAvoidingObject = false;

  private void toggleFieldRelative() {
    isFieldRelative = !isFieldRelative;
  }

  private void toggleTracking() {
    if (!isAvoidingObject) {
        isTrackingObject = !isTrackingObject;
    }
  }

  private void toggleObjectAvoidance() {
    isAvoidingObject = !isAvoidingObject;
    isTrackingObject = true;
  }

  private void toggleBalancing() {
    isBalancing = !isBalancing;
    if (isBalancing) {
        isAvoidingObject = false;
        isTrackingObject = false;
        isFieldRelative = true;
    }
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller = new XboxController(0);
    m_joystick = new Joystick(1);

    new JoystickButton(m_controller, Button.kB.value)
      .toggleOnTrue(new InstantCommand(
        this::toggleBalancing));

    new JoystickButton(m_controller, Button.kA.value)
      .toggleOnTrue(new InstantCommand(
        m_driveSystem::zeroHeading, m_driveSystem));

    new JoystickButton(m_controller, Button.kY.value)
      .toggleOnTrue(new InstantCommand(
        this::toggleFieldRelative));

    new JoystickButton(m_controller, Button.kX.value)
      .toggleOnTrue(new InstantCommand(
        this::toggleTracking));

    // new JoystickButton(m_controller, Button.kLeftBumper.value)
    //   .toggleOnTrue(new InstantCommand(
    //     m_driveSystem::toggleCam, m_driveSystem));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSystem.drive(
      -MathUtil.applyDeadband(m_controller.getLeftY()/5,
        OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(
        m_controller.getLeftX()/5,
        OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(
        m_controller.getRightX()/2,
        OIConstants.kDriveDeadband),
      isFieldRelative, true, isTrackingObject,
      isAvoidingObject, isBalancing);
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
