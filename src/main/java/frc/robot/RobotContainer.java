package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.choreo.lib.*;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.MechanismCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.ModuleConstants;

public class RobotContainer {
    private static final LimeLight m_limeLight = new LimeLight();
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem(m_limeLight);
    private static final MechanismSubsystem m_mechSystem = new MechanismSubsystem();
    private static final DriveCommand m_driveCommand = new DriveCommand(m_driveSystem);
    private static final MechanismCommand m_mechCommand = new MechanismCommand(m_mechSystem);
    private static final ParallelCommandGroup m_teleopCommand = new ParallelCommandGroup(m_driveCommand, m_mechCommand);
    private static final ChoreoTrajectory autoPath = Choreo.getTrajectory("test");
    private static final Command m_autoCommand = Choreo.choreoSwerveCommand(
        autoPath, 
        m_driveSystem::getPose,  
        new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD), 
        new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD), 
        new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD), 
        (ChassisSpeeds speeds) ->
            m_driveSystem.drive(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond, 
                speeds.omegaRadiansPerSecond, 
                true, true, false, false, false),
        () -> { 
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == Alliance.Red;
        }, 
        m_driveSystem);

    public Command getTeleopCommand() {
        return m_teleopCommand;
    }

    public Command getAutoCommand() {
        return m_autoCommand;
    }
}
