package frc.robot;

public final class Constants
{
    public enum AutoMode {
        amNothing,
        amLaunchDrive,
        amLaunch,
        amDrive,
    }
    public static class DriveConstants
    {
        public static final int DRIVE_CURRENT_LIMIT_A = 60;

        /*
         * MOTOR IDS
         * to set when motor IDs are determied (lol)
         */
        public static final int LEFT_REAR_MOTOR_ID = 0;
        public static final int LEFT_FRONT_MOTOR_ID = 0;
        public static final int RIGHT_REAR_MOTOR_ID = 0;
        public static final int RIGHT_FRONT_MOTOR_ID = 0;
        public static final int CLIMBER_MOTOR_ID = 0;

        // default climber speed (speed at which climber retracts/expands)
        // to set to whatever feels right
        // also can be split to expand and retract speeds separately if needed
        public static final double CLIMBER_MOTOR_SPEED = 0;
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
}
