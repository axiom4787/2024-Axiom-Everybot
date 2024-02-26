package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    // private static final SendableChooser<String> autoChooser = new SendableChooser<>();


    public RobotContainer() {
        m_driveSystem.setupPathPlanner();
        NamedCommands.registerCommand("shoot", m_autoShoot);
        NamedCommands.registerCommand("stop", m_autoStop);
        configureBindings();
    }

    public Command getTeleopCommand() {
        return m_teleopCommand;
    }

    private void configureBindings() {
        // SmartDashboard.putData("Auto Mode", autoChooser);
        // autoChooser.addOption("close", "amp shoot close");
        // autoChooser.addOption("center", "amp shoot center");
        // autoChooser.addOption("far", "amp shoot far");
    }

    public Command getAutoCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("test");
        m_driveSystem.resetOdometry(new Pose2d(path.getPoint(0).position, new Rotation2d(m_driveSystem.getHeading())));
        return AutoBuilder.followPath(path);
    }
}
