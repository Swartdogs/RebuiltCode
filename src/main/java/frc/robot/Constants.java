package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants
{
    public static final Mode SIM_MODE     = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public static enum Mode
    {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class AIO
    {
        public static final int FL_ENCODER = 0;
        public static final int FR_ENCODER = 1;
        public static final int BL_ENCODER = 2;
        public static final int BR_ENCODER = 3;
    }

    public static class CAN
    {
        public static final int FL_DRIVE = 1;
        public static final int FR_DRIVE = 3;
        public static final int BL_DRIVE = 5;
        public static final int BR_DRIVE = 7;
        public static final int FL_TURN  = 2;
        public static final int FR_TURN  = 4;
        public static final int BL_TURN  = 6;
        public static final int BR_TURN  = 8;
        public static final int INTAKE   = 20;
    }

    public static class Drive
    {
        
    }

    public static class Intake 
    {
        public static final double INTAKE_VOLTS  = 8.0;
        public static final double REVERSE_VOLTS  = 4.0;
        public static final int    CURRENT_LIMIT = 40;
    }

    public static class General
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MOTOR_VOLTAGE = 12.0;
    }
}