package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LaunchFeedSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.ModuleConstants;
import com.choreo.lib.*;

public class RobotContainer {
    private static final LimeLight m_limeLight = new LimeLight();
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem(m_limeLight);
    private static final LaunchFeedSubsystem m_mechSystem = new LaunchFeedSubsystem();
    private static final ClimberSubsystem m_climberSystem = new ClimberSubsystem();
    private static final AutoCommand m_autoCommand = new AutoCommand(m_driveSystem, m_mechSystem);
    private static final DriveCommand m_driveCommand = new DriveCommand(m_driveSystem, m_mechSystem, m_climberSystem);
//    private static final ChoreoTrajectory autoPath = Choreo.getTrajectory("test");

    public Command getTeleopCommand() {
        return m_driveCommand;
    }

    public Command getAutoCommand() {
        return m_autoCommand;
    }
}
