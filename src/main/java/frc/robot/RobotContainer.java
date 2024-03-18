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
import frc.robot.subsystems.ClimberSubsystem;
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
    private static final StadiaController m_gunnerController = new StadiaController(0);
    private static final XboxController m_driverController = new XboxController(1);
    private static final LimeLight m_limeLight = new LimeLight();
    private static final DriveSubsystem m_driveSystem = new DriveSubsystem(m_limeLight);
    private static final MechanismSubsystem m_mechSystem = new MechanismSubsystem();
    private static final ClimberSubsystem m_climberSystem = new ClimberSubsystem();
    private static final Command m_speakerPath = new InstantCommand(m_driveSystem::zeroHeading, m_driveSystem)
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mPositive), m_mechSystem))
        .andThen(new WaitCommand(4))
        .andThen(new InstantCommand(() -> m_mechSystem.setShooterState(MechState.mReset), m_mechSystem));
        // .andThen(new InstantCommand(() -> m_driveSyst    em.setChassisSpeeds(new ChassisSpeeds(-0.5, 0, 0)), m_driveSystem))
        // .andThen(new WaitCommand(3))
        // .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds()), m_driveSystem));
    private static final Command m_blueAmpPath = new InstantCommand(m_driveSystem::zeroHeading, m_driveSystem)
        .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds(0.25, 0.25, 0)), m_driveSystem))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds()), m_driveSystem))
        .andThen(new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mPositive), m_mechSystem))
        .andThen(new WaitCommand(2))
        .andThen(new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mReset), m_mechSystem))
        .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds(-0.25, 0.25, 0)), m_driveSystem))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds()), m_driveSystem));
    private static final Command m_redAmpPath = new InstantCommand(m_driveSystem::zeroHeading, m_driveSystem)
        .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds(0.25, -0.25, 0)), m_driveSystem))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds()), m_driveSystem))
        .andThen(new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mPositive), m_mechSystem))
        .andThen(new WaitCommand(2))
        .andThen(new InstantCommand(() -> m_mechSystem.setRollerState(MechState.mReset), m_mechSystem))
        .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds(-0.25, -0.25, 0)), m_driveSystem))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_driveSystem.setChassisSpeeds(new ChassisSpeeds()), m_driveSystem));

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
        m_climberSystem.setClimberSpeed(-m_gunnerController.getRightY());
    }, m_driveSystem);

    private static final ChoreoTrajectory traj = Choreo.getTrajectory("test");

    private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    public RobotContainer() {
        m_driveSystem.setupPathPlanner();
        m_autoChooser.addOption("speaker", m_speakerPath);
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean blue = alliance.isPresent() && alliance.get() == Alliance.Blue;
        m_autoChooser.addOption("amp", blue ? m_blueAmpPath : m_redAmpPath);
        configureBindings();
    }

    public Command getTeleopCommand() {
        return m_teleopCommand;
    }

    private void configureBindings() {
        // binds left bumper to enable shooters and roller for scoring
        Trigger leftBumperState = new JoystickButton(m_gunnerController, StadiaController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setShooterState(MechState.mPositive);
        }));

        // binds right bumper to enable shooters and roller for intake
        Trigger rightBumperState = new JoystickButton(m_gunnerController, StadiaController.Button.kRightBumper.value)
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
        Trigger leftTriggerState = new JoystickButton(m_gunnerController, StadiaController.Button.kLeftTrigger.value)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setRollerState(MechState.mPositive);
        }));

        // binds right bumper to enable shooters and roller for intake
        Trigger rightTriggerState = new JoystickButton(m_gunnerController, StadiaController.Button.kRightTrigger.value)
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setRollerState(MechState.mNegative);
        }));

        // if both left and right triggers are disabled, then reset the roller
        leftTriggerState.or(rightTriggerState)
        .negate()
        .onTrue(new InstantCommand(() -> {
            m_mechSystem.setRollerState(MechState.mReset);
        }));

        // // binds y button to retract climbers
        // Trigger yState = new JoystickButton(m_gunnerController, StadiaController.Button.kY.value)
        // .onTrue(new InstantCommand(() -> {
        //     m_climberSystem.setClimberState(MechState.mNegative);
        // }));

        // // binds b button to extend climbers
        // Trigger bState = new JoystickButton(m_gunnerController, StadiaController.Button.kB.value)
        // .onTrue(new InstantCommand(() -> {
        //     m_climberSystem.setClimberState(MechState.mPositive);
        // }));

        // // if neither y/b pressed do nothing with climbers
        // yState.or(bState)
        // .negate()
        // .onTrue(new InstantCommand(() -> {
        //     m_climberSystem.setClimberState(MechState.mReset);
        // }));

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
 