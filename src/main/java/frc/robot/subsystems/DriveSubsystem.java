// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.kauailabs.navx.frc.AHRS;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
    private LimeLight LL;

    private double prevRot;
    private boolean hasPrevRot = false;
    private PIDController headingController = new PIDController(
        DriveConstants.kHeadingP, DriveConstants.kHeadingI, DriveConstants.kHeadingD);
    private double gyroOffset;
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    private double m_frontLeftPosition = m_frontLeft.getPosition().distanceMeters;
    private double m_frontRightPosition = m_frontRight.getPosition().distanceMeters;
    private double m_rearLeftPosition = m_rearLeft.getPosition().distanceMeters;
    private double m_rearRightPosition = m_rearRight.getPosition().distanceMeters;

    // Field object for Smart Dashboard
    private final Field2d m_field;

    // The gyro sensor
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP); // navX

    private boolean isFieldRelative = true;
    private boolean isStatue = false;
    private boolean isBalancing = false;
    private boolean isTrackingObject = false; // allows for rotation to be controlled by the limelight
    private boolean isAvoidingObject = false;
    private boolean isAimAssist = false; // allows for rotation to be controlled by the controller and the limelight
                                        // //requires isTrackingObject to be true

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    public double speedReduction = 1;

    PIDController trackingPID = new PIDController(DriveConstants.kTrackingP, DriveConstants.kTrackingI,
            DriveConstants.kTrackingD);
    PIDController avoidingPID = new PIDController(DriveConstants.kAvoidingP, DriveConstants.kAvoidingI,
            DriveConstants.kAvoidingD);

    // Odometry class for tracking robot pose
    SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(-m_gyro.getYaw()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            },
            new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0))
    );

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(LimeLight LL) {
        this.LL = LL;
        zeroHeading();
        // m_gyro.zeroYaw();

        trackingPID.setTolerance(DriveConstants.kTrackingTolerance);
        trackingPID.setIntegratorRange(DriveConstants.kTrackingIntergratorRangeMin,
                DriveConstants.kTrackingIntergratorRangeMax);
        headingController.enableContinuousInput(-180, 180);
        headingController.setTolerance(0.05);
        m_field = new Field2d();
        m_field.getObject("Speaker Pose").setPose(AlignConstants.kSpeakerPose);
        m_field.getObject("Amp Pose").setPose(AlignConstants.kAmpPose);
        m_field.getObject("Source Close Pose").setPose(AlignConstants.kSourceClosePose);
        m_field.getObject("Source Center Pose").setPose(AlignConstants.kSourceCenterPose);
        m_field.getObject("Source Far Pose").setPose(AlignConstants.kSourceFarPose);

    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        double xSpeed = speeds.vxMetersPerSecond;
        double ySpeed = speeds.vyMetersPerSecond;
        double rot = speeds.omegaRadiansPerSecond;
        double[] args = {xSpeed, ySpeed, rot};
        SmartDashboard.putNumberArray("SetChassisSpeeds Calls", args);
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void toggleFieldRelative() {
        isFieldRelative = !isFieldRelative;
    }

    public void toggleStatue() {
        isStatue = !isStatue;
    }

    /**
     * Gets the current robot-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current robot-relative velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        );
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner()
    {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                                                                new PIDConstants(3, 0.0, 0), // P: 5 I: 0.0 D: 0.0
                                                                                // Translation PID constants
                                                                                new PIDConstants(1, 0, 0),
                                                                                // Rotation PID constants
                                                                                4.12,
                                                                                // Max module speed, in m/s
                                                                                0.15909902576697319299018,
                                                                                // Drive base radius in meters. Distance from robot center to furthest module.
                                                                                new ReplanningConfig()
                                                                                // Default path replanning config. See the API for the options here
                ),
                () -> {
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == Alliance.Red;
                },
                this // Reference to this subsystem to set requirements
                                                                    );
    }

    public void toggleCam() {
        if (LL.getPipeline() == 7.0) {
            LL.setPipeline(8);
        } else {
            LL.setPipeline(7);
        }
    }

    public void updateShuffleBoard() {
        SmartDashboard.putNumber("Gyro Angle", Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset);
        SmartDashboard.putNumber("Gyro Roll", Rotation2d.fromDegrees(m_gyro.getRoll()).getDegrees());
        SmartDashboard.putBoolean("Field Relative", isFieldRelative);
        SmartDashboard.putBoolean("Tracking Object", isTrackingObject);
        SmartDashboard.putBoolean("Avoiding Object", isAvoidingObject);
        SmartDashboard.putBoolean("Balancing", isBalancing);
        SmartDashboard.putNumber("stupid", convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset));
        SmartDashboard.putNumber("Gyro Offset", gyroOffset);

        m_frontLeftPosition = m_frontLeft.getPosition().distanceMeters;
        m_frontRightPosition = m_frontRight.getPosition().distanceMeters;
        m_rearLeftPosition = m_rearLeft.getPosition().distanceMeters;
        m_rearRightPosition = m_rearRight.getPosition().distanceMeters;

        SmartDashboard.putNumber("FR Odometry", m_frontRightPosition);
        SmartDashboard.putNumber("FL Odometry", m_frontLeftPosition);
        SmartDashboard.putNumber("BR Odometry", m_rearRightPosition);
        SmartDashboard.putNumber("BL Odometry", m_rearLeftPosition);
        SmartDashboard.putNumber("X Translation Drive", m_odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y Translation Drive", m_odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Yaw Rotation Drive", m_odometry.getEstimatedPosition().getRotation().getDegrees());

        m_field.setRobotPose(getPose());

        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        updateShuffleBoard();
        // Update the odometry in the periodic block
        m_odometry.update(
                Rotation2d.fromDegrees(-m_gyro.getYaw() + gyroOffset),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
        m_odometry.addVisionMeasurement(LL.getBotPose2d(), /*LL.getLocalizationLatency()*/0.03);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        /*if (LL.getBotPose2d() != null || (LL.getBotPose2d().getX() != 0 && LL.getBotPose2d().getY() != 0)) {
            return LL.getBotPose2d();
        }*/
        return m_odometry.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(-m_gyro.getYaw() + gyroOffset),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param _xSpeed         Speed of the robot in the x direction (forward).
     * @param _ySpeed         Speed of the robot in the y direction (sideways).
     * @param ro/t            Angular rate of the robot.
     */
    public void drive(double _xSpeed, double _ySpeed, double rot) {
        if (isStatue)
        {
            setX();
            return;
        }
        
        double xSpeed = _xSpeed;
        double ySpeed = _ySpeed;

        xSpeed = xSpeed / speedReduction;
        ySpeed = ySpeed / speedReduction;
//        rot = rot / speedReduction;

        // changes xspeed and yspeed based off of the navx gyro to stop the robot from tipping over only from the front

        double xSpeedCommanded;
        double ySpeedCommanded;

        // Convert XY to polar for rate limiting
        double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

        // Calculate the direction slew rate based on an estimate of the lateral
        // acceleration
        double directionSlewRate;
        if (m_currentTranslationMag != 0.0) {
            directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
        } else {
            directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
        }

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
        if (angleDif < 0.45 * Math.PI) {
            m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                    directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        } else if (angleDif > 0.85 * Math.PI) {
            if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                    // checking
                // keep currentTranslationDir unchanged
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            } else {
                m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            }
        } else {
            m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                    directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        m_prevTime = currentTime;

        xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
        ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
        //m_currentRotation = m_rotLimiter.calculate(trackingObject ? calculateTrackingAngularVelocity(rot) : balancing ? calculateBalanceCenterTrackingAngularVelocity(rot) : rot);
        // if (!hasPrevRot)
        // {
        //     m_currentRotation = m_rotLimiter.calculate(rot);
        // }
        // else if (prevRot != 0 && rot == 0)
        // {
        //     headingController.reset();
        //     headingController.setSetpoint(getPose().getRotation().getDegrees());
        //     m_currentRotation = m_rotLimiter.calculate(rot);
        // }
        // else if (rot == 0)
        // {
        //     double desiredRot = MathUtil.clamp(headingController.calculate(
        //         getPose().getRotation().getDegrees()), -1, 1);
        //     m_currentRotation = m_rotLimiter.calculate(desiredRot);
        // }
        // else
        // {
            m_currentRotation = m_rotLimiter.calculate(rot);
//        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                this.isFieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(-m_gyro.getYaw() + gyroOffset))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
        prevRot = rot;
        if (prevRot != 0)
        {
            hasPrevRot = true;
        }
    }

    private double convertToRange(double degrees) {
        degrees = degrees % 360; // Convert degrees to the range of -360 to 360

        if (degrees > 180) {
            degrees -= 360; // Adjust degrees greater than 180 to the corresponding negative range
        } else if (degrees < -180) {
            degrees += 360; // Adjust degrees less than -180 to the corresponding positive range
        }

        return degrees;
    }

    public double calculateBalanceRollAdjustmentVelocity(double ySpeed) {
        double speedReduction = 2;
        SmartDashboard.putNumber("roll", m_gyro.getRoll());

        if (!(m_gyro.getRoll() <= 2 && m_gyro.getRoll() >= -2)) {
            return trackingPID.calculate(Rotation2d.fromDegrees(m_gyro.getRoll()).getDegrees(), 0) / speedReduction;
        }

        return 0;
    }

    public double calculateBalanceCenterTrackingAngularVelocity() {
        double speedReduction = 2;

        if (!(convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset) >= -0.5 && convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset) <= 0.5)) {
            return trackingPID.calculate(convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset), 0) / speedReduction;
        }

        return 0;
    }

    public double calculateTurnTo180AngularVelocity() {
        double speedReduction = 2;
        double setPoint = convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset) / Math.abs(convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset));
        SmartDashboard.putNumber("test", convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset));
        if (!(convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset) >= 178 || convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset) <= -178)) {
            return trackingPID.calculate(convertToRange(Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset),setPoint * 180) / speedReduction;
        }

        return 0;
    }

    public double calculateTrackingAngularVelocity(double rot) {
        /*
         * if (LL.getXAngle() != 0 && Math.abs(LL.getXAngle()) >= 1) {
         * double speed = 0.03; // between 0 amd 1
         * double direction = (-LL.getXAngle()) / Math.abs(LL.getXAngle());
         * double scaleFactor = (Math.abs(LL.getXAngle())) * speed;
         * SmartDashboard.putNumber("tracking velocity", direction * scaleFactor);
         * if (scaleFactor > 2) {
         * scaleFactor = 1.4;
         * }
         * return direction * scaleFactor;
         * }
         * 
         * return 0;
         */
        if (isAimAssist) {
            if (rot != 0) {
                return rot;
            }
        }

        if (LL.getXAngle() != 0) {
            return trackingPID.calculate(LL.getXAngle(), 0);
        }

        return 0;
    }

    public double calculateObjectAvoidanceVelocity(double velocity) {
        if (LL.getYAngle() >= 10) {
            return -(LL.getYAngle() - 10) / 10;
        }

        return velocity;
    }

    public void setCounterMovement(int setting) {
        if (setting == 1) { // getting pushed from the front
            m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
            m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
            m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
            m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        } else {
            m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
            m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
            m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
            m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
        }
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    public void setZeros() {
        m_frontLeft.setZero();
        m_rearLeft.setZero();
        m_frontRight.setZero();
        m_rearRight.setZero();
    }

    public void setOffsetZeros() {
        m_frontLeft.setOffsetZero();
        m_rearLeft.setOffsetZero();
        m_frontRight.setOffsetZero();
        m_rearRight.setOffsetZero();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
//        gyroOffset = -Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees();
        System.out.println(gyroOffset);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees() + gyroOffset;
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /*public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                        new PIDController(0.3, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
                                                    // will only use feedforwards.
                        new PIDController(0.3, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving
                                                    // them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer
                        true, // Should the path be automatically mirrored depending on alliance color.
                              // Optional, defaults to true
                        this // Requires this drive subsystem
                ));
    }*/
}