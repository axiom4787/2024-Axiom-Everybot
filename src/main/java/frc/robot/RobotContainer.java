package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.StadiaController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.BlinkinLights;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.MechState;
import frc.robot.Constants.OIConstants;

import java.util.List;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
    private static final Joystick m_gunnerController = new Joystick(0);
    private static final XboxController m_driverController = new XboxController(1);
    private static final LimeLight m_limeLight = new LimeLight();
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem(m_limeLight);
    private static final BlinkinLights m_lights = new BlinkinLights();
    private static final MechanismSubsystem m_mechSystem = new MechanismSubsystem(m_lights);
    private static final ClimberSubsystem m_climberSystem = new ClimberSubsystem(m_lights);
    private static final Command m_speakerPath = new InstantCommand(m_driveSystem::zeroHeading, m_driveSystem)
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mPositive), m_mechSystem))
        .andThen(new WaitCommand(4))
        .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mReset), m_mechSystem));
        // .andThen(new InstantCommand(() -> m_driveSyst    em.setChassisSpeeds(new ChassisSpeeds(-0.5, 0, 0)), m_driveSystem))
        // .andThen(new WaitCommand(3))
        // .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds()), m_driveSystem));

    // commands for aligning to speaker, amp, and source
    // ideally should only be run when very close to the target, to avoid collisions and pathing errors
    // can be canceled with the Y button
    private static final Command m_alignSpeaker = AutoBuilder.pathfindToPoseFlipped(AlignConstants.kSpeakerPose, AlignConstants.kAlignConstraints)
    .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mPositive), m_mechSystem))
    .andThen(new WaitCommand(2.5))
    .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mReset), m_mechSystem))
    .until(() -> m_driverController.getYButtonPressed());
    private static final Command m_alignAmp = AutoBuilder.pathfindToPoseFlipped(AlignConstants.kAmpPose, AlignConstants.kAlignConstraints)
    .andThen(new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mPositive), m_mechSystem))
    .andThen(new WaitCommand(2.5))
    .andThen(new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mReset), m_mechSystem))
    .until(() -> m_driverController.getYButtonPressed());
    private static final Command m_alignSourceClose = AutoBuilder.pathfindToPoseFlipped(AlignConstants.kSourceClosePose, AlignConstants.kAlignConstraints)
    .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mNegative), m_mechSystem))
    .andThen(new WaitCommand(2))
    .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mReset), m_mechSystem))
    .until(() -> m_driverController.getYButtonPressed());
    private static final Command m_alignSourceCenter = AutoBuilder.pathfindToPoseFlipped(AlignConstants.kSourceCenterPose, AlignConstants.kAlignConstraints)
    .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mNegative), m_mechSystem))
    .andThen(new WaitCommand(2))
    .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mReset), m_mechSystem))
    .until(() -> m_driverController.getYButtonPressed());
    private static final Command m_alignSourceFar = AutoBuilder.pathfindToPoseFlipped(AlignConstants.kSourceFarPose, AlignConstants.kAlignConstraints)
    .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mNegative), m_mechSystem))
    .andThen(new WaitCommand(2))
    .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mReset), m_mechSystem))
    .until(() -> m_driverController.getYButtonPressed());

    private static final Command m_teleopCommand = new RunCommand(() -> {
        double xSpeed, ySpeed, rot = 0;
        if (m_driverController.getPOV() != -1) {
            double POV = Math.toRadians(-m_driverController.getPOV());
            xSpeed = OIConstants.kDriveAdjustSpeed*Math.cos(POV);
            ySpeed = OIConstants.kDriveAdjustSpeed*Math.sin(POV); 
        }
        else {
            xSpeed = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
            ySpeed = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
        }
        if (m_driverController.getBackButton())
            rot = -OIConstants.kTurnAdjustSpeed;
        else if (m_driverController.getStartButton())
            rot = OIConstants.kTurnAdjustSpeed;
        else 
            rot = -MathUtil.applyDeadband(m_driverController.getRightX()*0.8, OIConstants.kDriveDeadband);
        m_driveSystem.drive(xSpeed, ySpeed, rot);
        m_climberSystem.setClimberSpeed(-m_gunnerController.getY());
    }, m_driveSystem);

    private static final ChoreoTrajectory traj = Choreo.getTrajectory("test");

    public RobotContainer() {
        m_driveSystem.setupPathPlanner();
        configureBindings();
    }

    public Command getTeleopCommand() {
        return m_teleopCommand;
    }

    private void configureBindings() {
        Trigger speakerShootControl = new JoystickButton(m_gunnerController, 1)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mPositive);
        }));

        Trigger speakerIntakeControl = new JoystickButton(m_gunnerController, 2)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mNegative);
        }));

        speakerShootControl.or(speakerIntakeControl)
        .negate()
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mReset);
        }));

        Trigger ampShootControl = new JoystickButton(m_gunnerController, 3)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setRollerState(MechState.mPositive);
        }));

        Trigger ampIntakeControl = new JoystickButton(m_gunnerController, 4)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setRollerState(MechState.mNegative);
        }));

        ampShootControl.or(ampIntakeControl)
        .negate()
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setRollerState(MechState.mReset);
        }));

        // zeroes gyro heading
        new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> {
            m_driveSystem.zeroHeading();
        }));

        // sets/unsets robot defending X pattern
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(() -> {
            m_driveSystem.toggleStatue();
        }));

        // toggles robot field relative
        new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(() -> {
            m_driveSystem.toggleFieldRelative();
        }));

    }

    public Command getAutoCommand() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("shoot");
    //     m_driveSystem.resetOdometry(path.getPreviewStartingHolonomicPose());
    //     return AutoBuilder.followPath(path);
        // m_driveSystem.resetOdometry(traj.getInitialPose());
        // return Choreo.choreoSwerveCommand(
        //     traj, 
        //     m_driveSystem::getPose, 
        //     new PIDController(2.5, 0, 0), 
        //     new PIDController(2.5, 0, 0),
        //     new PIDController(1.35, 0, 0),
        //     m_driveSystem::setChassisSpeeds,
        //     () -> {
        //         Optional<Alliance> alliance = DriverStation.getAlliance();
        //         return alliance.isPresent() && alliance.get() == Alliance.Red;
        //     },
        //     m_driveSystem);
        return m_speakerPath;
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
 