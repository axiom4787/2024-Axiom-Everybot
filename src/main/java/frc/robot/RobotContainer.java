package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.MechanismCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.ModuleConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
    private static final LimeLight m_limeLight = new LimeLight();
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem(m_limeLight);
    private static final MechanismSubsystem m_mechSystem = new MechanismSubsystem();
    private static final DriveCommand m_driveCommand = new DriveCommand(m_driveSystem);
    private static final MechanismCommand m_mechCommand = new MechanismCommand(m_mechSystem);
    private static final ParallelCommandGroup m_teleopCommand = new ParallelCommandGroup(m_driveCommand, m_mechCommand);
    private static final Command m_autoShoot = new InstantCommand(() -> m_mechSystem.ejectNote(), m_mechSystem);
    private static final Command m_autoStop = new InstantCommand(() -> m_mechSystem.resetMotors(), m_mechSystem);


    public RobotContainer() {
        NamedCommands.registerCommand("shoot", m_autoShoot);
        NamedCommands.registerCommand("stop", m_autoStop);
        configureBindings();
    }

    public Command getTeleopCommand() {
        return m_teleopCommand;
    }

    private void configureBindings() {
        SmartDashboard.putData("amp close", AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("amp shoot close")
        ));
        SmartDashboard.putData("amp center", AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("amp shoot center")
        ));
        SmartDashboard.putData("amp far", AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("amp shoot far")
        ));
    }

    // public Command getAutoCommand() {
    //     return m_autoCommand;
    // }
}
