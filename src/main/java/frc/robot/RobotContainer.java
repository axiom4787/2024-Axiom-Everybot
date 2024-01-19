package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.Constants.AutoMode;

public class RobotContainer {
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem();
    private static final MechanismSubsystem m_mechSystem = new MechanismSubsystem();
    private final SendableChooser<AutoMode> m_chooser;
    private final AutoCommand m_autoCommand;
    private final DriveCommand m_driveCommand;
    public RobotContainer(SendableChooser<AutoMode> autoChooser)
    {
        m_chooser = autoChooser;
        m_autoCommand = new AutoCommand(m_driveSystem, m_mechSystem, m_chooser);
        m_driveCommand = new DriveCommand(m_driveSystem, m_mechSystem);
    }

    public Command getTeleopCommand() {
        return m_driveCommand;
    }

    public Command getAutoCommand() {
        return m_autoCommand;
    }
}
