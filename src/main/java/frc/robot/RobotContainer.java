package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.StadiaController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.MechState;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoTestCommand;

import java.util.List;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
    private static final StadiaController m_controller = new StadiaController(0);
    private static final LimeLight m_limeLight = new LimeLight();
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem(m_limeLight);
    private static final MechanismSubsystem m_mechSystem = new MechanismSubsystem();
    private static final Command m_autoAmpShoot = new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mPositive), m_mechSystem);
    private static final Command m_autoRollerReset = new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mReset), m_mechSystem);
    private static final Command m_autoSpeakerShoot = new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mPositive), m_mechSystem);
    private static final Command m_autoShooterReset = new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mReset), m_mechSystem);
    private static final Command m_driveCommand = new RunCommand(() -> {
        double xSpeed, ySpeed, rot;
        if (m_controller.getPOV() != -1) {
            double POV = Math.toRadians(-m_controller.getPOV());
            xSpeed = OIConstants.kDriveAdjustSpeed*Math.cos(POV);
            ySpeed = OIConstants.kDriveAdjustSpeed*Math.sin(POV); 
        }
        else {
            xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband);
            ySpeed = -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband);
        }
        if (m_controller.getEllipsesButton())
            rot = -OIConstants.kTurnAdjustSpeed;
        else if (m_controller.getHamburgerButton())
            rot = OIConstants.kTurnAdjustSpeed;
        else
            rot = -MathUtil.applyDeadband(m_controller.getRightX()/2, OIConstants.kDriveDeadband);
        m_driveSystem.drive(xSpeed, ySpeed, rot);
    }, m_driveSystem);

    private static final ChoreoTrajectory traj = Choreo.getTrajectory("test");

    private static final Command m_autoTestCommand = new AutoTestCommand(m_driveSystem); 

    public RobotContainer() {
        m_driveSystem.setupPathPlanner();
        NamedCommands.registerCommand("amp shoot", m_autoAmpShoot);
        NamedCommands.registerCommand("amp stop", m_autoRollerReset);
        NamedCommands.registerCommand("speaker shoot", m_autoSpeakerShoot);
        NamedCommands.registerCommand("speaker stop", m_autoShooterReset);
        configureBindings();
    }

    public Command getTeleopCommand() {
        return m_driveCommand;
    }

    private void configureBindings() {
        // binds left bumper to enable shooters and roller for scoring
        Trigger leftBumperState = new JoystickButton(m_controller, StadiaController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mPositive);
        }));

        // binds right bumper to enable shooters and roller for intake
        Trigger rightBumperState = new JoystickButton(m_controller, StadiaController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mNegative);
        }));

        // if both left and right bumpers are disabled, then reset the shooters and roller
        leftBumperState.or(rightBumperState)
        .negate()
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mReset);
        }));

        // binds left bumper to enable shooters and roller for scoring
        Trigger leftTriggerState = new JoystickButton(m_controller, StadiaController.Button.kLeftTrigger.value)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setRollerState(MechState.mPositive);
        }));

        // binds right bumper to enable shooters and roller for intake
        Trigger rightTriggerState = new JoystickButton(m_controller, StadiaController.Button.kRightTrigger.value)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setRollerState(MechState.mNegative);
        }));

        // if both left and right bumpers are disabled, then reset the shooters and roller
        leftTriggerState.or(rightTriggerState)
        .negate()
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setRollerState(MechState.mReset);
        }));

        new JoystickButton(m_controller, StadiaController.Button.kB.value)
        .onTrue(new InstantCommand(() -> {
            m_driveSystem.toggleFieldRelative();
        }));

        new JoystickButton(m_controller, StadiaController.Button.kA.value)
        .onTrue(new InstantCommand(() -> {
            m_driveSystem.zeroHeading();
        }));

        new JoystickButton(m_controller, StadiaController.Button.kX.value)
        .onTrue(new InstantCommand(() -> {
            m_driveSystem.toggleStatue();
        }));

        new JoystickButton(m_controller, StadiaController.Button.kFrame.value)
        .onTrue(new InstantCommand(() -> {
            m_driveSystem.speedReduction = m_driveSystem.speedReduction == 4 ? 4 : 4;
        }));
    }

    public Command getAutoCommand() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("shoot");
    //     m_driveSystem.resetOdometry(path.getPreviewStartingHolonomicPose());
    //     return AutoBuilder.followPath(path);
        m_driveSystem.resetOdometry(traj.getInitialPose());
        return Choreo.choreoSwerveCommand(
            traj, 
            m_driveSystem::getPose, 
            new PIDController(2.5, 0, 0), 
            new PIDController(2.5, 0, 0),
            new PIDController(1.35, 0, 0),
            m_driveSystem::setChassisSpeeds,
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            },
            m_driveSystem);
    }

    public Command getZeroCommand() {
        return m_driveSystem.runOnce(() -> {
            Pose2d pose = m_driveSystem.getPose();
            System.out.println(pose.getX());
            System.out.println(pose.getY());
            System.out.println(pose.getRotation().getDegrees());
        })
        .andThen(new RunCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds()), m_driveSystem));
    }
}
