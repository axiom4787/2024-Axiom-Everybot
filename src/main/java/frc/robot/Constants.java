package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import org.opencv.features2d.KAZE;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

public final class Constants
{
    public enum MechState
    {
        mReset,
        mPositive,
        mNegative,
    }

    public static class BlinkinConstants
    {
        public static final double kIntaking = -0.09;
        public static final double kShooting = -0.11;
        public static final double kFull = 0.73;
        public static final double kClimbing = 0.91;
        public static final double kEmpty = 0.95;
    }

    public static class LimelightConstants {
        public static final double kLLHeight = Units.inchesToMeters(20);
        public static final double kLLPitch = Units.degreesToRadians(0);
        public static final double kMinObjectAvoidanceDistance = Units.inchesToMeters(12);
        public static final double kObjectHeight = Units.inchesToMeters(57);
        public static final double kObjectPitch = Units.degreesToRadians(0);
    }

    public static class AlignConstants {
        public static final double kRobotWidth = Units.inchesToMeters(26);
        public static final double kBumperWidth = Units.inchesToMeters(3.25);
        public static final double kFullWidth = kRobotWidth + 2 * kBumperWidth;
        public static final double kHalfWidth = kFullWidth / 2;
        public static final Pose2d kSpeakerPose = new Pose2d(1.356747, 5.556003, new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d kAmpPose = new Pose2d(1.637335, 7.736592, new Rotation2d(Units.degreesToRadians(90)));
        public static final Pose2d kSourceFarPose = new Pose2d(15.974139, 1.240227, new Rotation2d(Units.degreesToRadians(-60)));
        public static final Pose2d kSourceCenterPose = new Pose2d(15.432775, 0.941544, new Rotation2d(Units.degreesToRadians(-60)));
        public static final Pose2d kSourceClosePose = new Pose2d(14.891412, 0.624193, new Rotation2d(Units.degreesToRadians(-60)));
        public static final PathConstraints kAlignConstraints = new PathConstraints(0.5, 0.5, Math.PI, Math.PI);
    }
    
    public static class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 40; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class Neo550MotorConstants {
        public static final double kFreeSpeedRpm = 11000;
    }

    public static class DriveConstants
    {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.12;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        public static final double kHeadingP = 0.025;
        public static final double kHeadingI = 0.003;
        public static final double kHeadingD = 0.001;

        public static final double kTrackingP = 0.02; // 0.02
        public static final double kTrackingI = 0.0; // 0.003
        public static final double kTrackingD = 0.0; // 0.001
        public static final double kTrackingTolerance = 0.01;
        public static final double kTrackingToleranceDerivative = 6;
        public static final double kTrackingIntergratorRangeMin = -0.7;
        public static final double kTrackingIntergratorRangeMax = 0.7;

        public static final double kAvoidingP = 0.02;
        public static final double kAvoidingI = 0;
        public static final double kAvoidingD = 0;
        public static final double kAvoidingTolerance = 0.01;
        public static final double kAvoidingToleranceDerivative = 6;
        public static final double kAvoidingIntergratorRangeMin = -0.7;
        public static final double kAvoidingIntergratorRangeMax = 0.7;

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(22.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 5; // neo
        public static final int kRearLeftDrivingCanId = 7; // neo
        public static final int kFrontRightDrivingCanId = 3; // neo
        public static final int kRearRightDrivingCanId = 9; // neo

        public static final int kFrontLeftTurningCanId = 6; // neo 550
        public static final int kRearLeftTurningCanId = 8; // neo 550
        public static final int kFrontRightTurningCanId = 4; // neo 550
        public static final int kRearRightTurningCanId = 2; // neo 550

        public static final boolean kGyroReversed = false;
    }

    public static final class ClimberConstants
    {
        public static final int kLeftCanId = 14; // neo
        public static final int kRightCanId = 15; // neo
        public static final double kClimberSpeed = 0.75;
    }

    public static final class ShooterConstants
    {
        public static final int kFrontCanId = 11; // neo
        public static final int kBackCanId = 12; // neo
        public static final int kIndexerCanId = 13; // redline
        public static final double kShooterLaunchSpeed = 1;//1;
        public static final double kShooterIntakeSpeed = -0.5;//-1;
        public static final double kIndexerLaunchSpeed = 1;//0.5;
        public static final double kIndexerIntakeSpeed = -0.5;//-0.5;
        public static final double kIndexerDelay = 1;
    }

    public static final class RollerConstants
    {
        public static final int kCanId = 10; // neo
        public static final double kRollerSpeed = 0.2;
    }

    public static final class OIConstants {
        public static final double kDriveAdjustSpeed = 0.1;
        public static final double kTurnAdjustSpeed = 0.15;
        public static final double kDriveDeadband = 0.1;
    }

    public static final class PPvar {
        public static final double squareSize = Units.inchesToMeters(48);
        public static final double figureEightSize = Units.inchesToMeters(24);
        public static final double starSize = Units.inchesToMeters(24);
        public static final double maxVel = 0.00001;
        public static final double maxAccel = 0.0000001;
    }
}
