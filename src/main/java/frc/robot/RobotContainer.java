package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.StadiaController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.MechState;
import frc.robot.Constants.OIConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
    private static final StadiaController m_controller = new StadiaController(0);
    private static final LimeLight m_limeLight = new LimeLight();
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem(m_limeLight);
    private static final MechanismSubsystem m_mechSystem = new MechanismSubsystem();
    private static final Command m_autoShoot = new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mPositive), m_mechSystem);
    private static final Command m_autoStop = new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mReset), m_mechSystem);
    private static final Command m_mechCommand = m_mechSystem.mechCommand();
    private static final Command m_driveCommand = new InstantCommand(() -> {
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
    private static final ParallelCommandGroup m_teleopCommand = new ParallelCommandGroup(m_driveCommand, m_mechCommand);


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

        // binds left bumper to enable shooters and roller for scoring
        Trigger leftBumperState = new JoystickButton(m_controller, StadiaController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mPositive);
            m_mechSystem.setRollerState(MechState.mPositive);
        }));

        // binds right bumper to enable shooters and roller for intake
        Trigger rightBumperState = new JoystickButton(m_controller, StadiaController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mNegative);
            m_mechSystem.setRollerState(MechState.mNegative);
        }));

        // if both left and right bumpers are disabled, then reset the shooters and roller
        leftBumperState.or(rightBumperState)
        .negate()
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mReset);
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
            m_driveSystem.speedReduction = m_driveSystem.speedReduction == 1.25 ? 2 : 1.25;
        }));
    }

    public Command getAutoCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("test");
        m_driveSystem.resetOdometry(new Pose2d(path.getPoint(0).position, new Rotation2d(m_driveSystem.getHeading())));
        return AutoBuilder.followPath(path);
    }
}
