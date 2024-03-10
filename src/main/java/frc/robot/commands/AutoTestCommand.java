// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTestCommand extends Command {
  /** Creates a new AutoTestCommand. */
  private final DriveSubsystem m_subsystem;
  private double startTime;
  public AutoTestCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    System.out.println(startTime);
    m_subsystem.setChassisSpeeds(new ChassisSpeeds(0.1, 0, 0));
    System.out.println("execute");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setChassisSpeeds(new ChassisSpeeds(0,0,0));
    System.out.println("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(Timer.getFPGATimestamp());
    if (Timer.getFPGATimestamp()-startTime >= 1) {
      return true;
    }
    return false;
  }
}
