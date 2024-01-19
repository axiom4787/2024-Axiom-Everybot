package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;

public class RobotContainer {
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem();
    private static final MechanismSubsystem m_mechSystem = new MechanismSubsystem();

    private static final DriveCommand m_driveCommand = new DriveCommand(m_driveSystem, m_mechSystem);

    public Command getTeleopCommand() {
        return m_driveCommand;
    }
}
