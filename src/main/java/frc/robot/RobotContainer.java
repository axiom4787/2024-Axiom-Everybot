package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MechanismCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.ModuleConstants;

import java.util.Optional;

import com.choreo.lib.*;

public class RobotContainer {
    private static final LimeLight m_limeLight = new LimeLight();
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem(m_limeLight);
    private static final MechanismSubsystem m_mechSystem = new MechanismSubsystem();
    //private static final AutoCommand m_autoCommand = new AutoCommand(m_driveSystem/*/, m_mechSystem/**/);
    private static final DriveCommand m_driveCommand = new DriveCommand(m_driveSystem);
    private static final MechanismCommand m_mechCommand = new MechanismCommand(m_mechSystem);
    private static final ChoreoTrajectory autoPath = Choreo.getTrajectory("test");

    public Command getTeleopCommand() {
//        return new ParallelCommandGroup(m_driveCommand, m_mechCommand);
        return m_mechCommand;
    }

    public Command getAutoCommand() {
        return Choreo.choreoSwerveCommand(
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
                    false, true, false, false, false),
            () -> { 
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            }, 
            m_driveSystem);
    }
}
