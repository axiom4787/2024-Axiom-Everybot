// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismSubsystem;

public class MechanismCommand extends Command {
  private final MechanismSubsystem m_subsystem;
  private XboxController m_controller;

  /** Creates a new MechanismCommand. */
  public MechanismCommand(MechanismSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller = new XboxController(0);
  }

  @Override
  public void execute() {
    if (m_controller.getLeftBumper())
      m_subsystem.ejectNote();
    else if (m_controller.getRightBumper())
      m_subsystem.intakeNote();
    else
      m_subsystem.resetMotors();

    // if (m_controller.getLeftTriggerAxis() > 0.8))
    //   m_subsystem.extendArms();
    // else if (m_controller.getRightTriggerAxis() > 0.8)
    //   m_subsystem.retractArms();
    // else
    //   m_subsystem.resetArmMotors();
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
