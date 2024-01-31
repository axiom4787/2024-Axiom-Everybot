package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants
{
    public enum AutoMode {
        amNothing,
        amLaunchDrive,
        amLaunch,
        amDrive,
    }

    public static class LimelightConstants {
        public static final double kLLHeight = Units.inchesToMeters(24.5);
        public static final double kLLPitch = Units.degreesToRadians(0);
        public static final double kMinObjectAvoidanceDistance = Units.inchesToMeters(12);
        public static final double kObjectHeight = Units.inchesToMeters(12);
        public static final double kObjectPitch = Units.degreesToRadians(0);

        public static final int[] kBlueAprilTags = new int[]{1,2,3,4};
        public static final int[] kRedAprilTags = new int[]{5,6,7,8};
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
        public static final double kWheelDiameterMeters = 0.0762;
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

        public static final int kDrivingMotorCurrentLimit = 30; // amps
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
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

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
        public static final double kTrackWidth = Units.inchesToMeters(18);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(18);
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
        public static final int kClimberCanId = 0;
        public static final int kFrontLeftDrivingCanId = 4; // neo
        public static final int kRearLeftDrivingCanId = 2; // neo
        public static final int kFrontRightDrivingCanId = 6; // neo
        public static final int kRearRightDrivingCanId = 8; // neo

        public static final int kFrontLeftTurningCanId = 3; // neo 550
        public static final int kRearLeftTurningCanId = 1; // neo 550
        public static final int kFrontRightTurningCanId = 5; // neo 550
        public static final int kRearRightTurningCanId = 7; // neo 550

        public static final int kLaunchCanId = 10;
        public static final int kFeedCanId = 9;

        public static final boolean kGyroReversed = false;

        // default climber speed (speed at which climber retracts/expands)
        // to set to whatever feels right
        // also can be split to expand and retract speeds separately if needed
        public static final double kClimberMotorSpeed = 0;
    }

    public static class FeederConstants
    {
        /**
         * How many amps the feeder motor can use.
         */
        public static final int FEEDER_CURRENT_LIMIT_A = 80;
      
        /**
         * Percent output to run the feeder when expelling note
         */
        public static final double FEEDER_OUT_SPEED = 1.0;
      
        /**
         * Percent output to run the feeder when intaking note
         */
        public static final double FEEDER_IN_SPEED = -.4;
      
        /**
         * Percent output for amp or drop note, configure based on polycarb bend
         */
        public static final double FEEDER_AMP_SPEED = .4;
      
        /**
         * How many amps the launcher motor can use.
         *
         * In our testing we favored the CIM over NEO, if using a NEO lower this to 60
         */
    }

    public static class LauncherConstants
    {
        public static final int LAUNCHER_CURRENT_LIMIT_A = 80;
      
        /**
         * Percent output to run the launcher when intaking AND expelling note
         */
        public static final double LAUNCHER_SPEED = 1.0;
      
        /**
         * Percent output for scoring in amp or dropping note, configure based on polycarb bend
         * .14 works well with no bend from our testing
         */
        public static final double LAUNCHER_AMP_SPEED = .17;
      
    }    

    public static final class OIConstants {
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
