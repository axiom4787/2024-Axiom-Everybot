// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismSubsystem;

public class MechanismCommand extends Command {
  private final MechanismSubsystem m_subsystem;
  private XboxController m_controller;
//  private Joystick m_joystick;

  /** Creates a new MechanismCommand. */
  public MechanismCommand(MechanismSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller = new XboxController(0);
//    m_joystick = new Joystick(1);
  }

  // FOR USE IN DEBUG WITH CONTROLLER ONLY
  @Override
  public void execute() {
    // if (m_controller.getLeftBumper())
    //   m_subsystem.intakeNote();
    // else if (m_controller.getRightBumper())
    //   m_subsystem.shootNote();
    // else
    //   m_subsystem.resetShooter();

    if (m_controller.getLeftBumper())
      m_subsystem.grabNote();
    else if (m_controller.getRightBumper())
      m_subsystem.dropNote();
    else
      m_subsystem.resetGrabber();

    // if (m_controller.getLeftTriggerAxis() > 0.5))
    //   m_subsystem.extendArms();
    // else if (m_controller.getRightTriggerAxis() > 0.5)
    //   m_subsystem.retractArms();
    // else
    //   m_subsystem.resetArmMotors();
  }

  // FOR USE IN COMPETITION WITH CONTROLLER AND JOYSTICK
  /*
  @Override
  public void execute()
  {
    if (m_joystick.getRawButton(5))
      m_subsystem.intakeNote();
    else if (m_joystick.getRawButton(3))
      m_subsystem.shootNote();
    else
      m_subsystem.resetShooter();

    if (m_joystick.getRawButton(6))
      m_subsystem.grabNote();
    else if (m_joystick.getRawButton(4))
      m_subsystem.dropNote();
    else
      m_subsystem.resetGrabber();

    if (m_joystick.getRawButton(1))
      m_subsystem.extendArms();
    else if (m_joystick.getRawButton(2))
      m_subsystem.retractArms();
    else
      m_subsystem.resetArmMotors();
  }
  */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
