package frc.robot;

public final class Constants
{
    public static class AIO
    {
    }

    public static class CAN
    {
        /* CAN IDs 1 through 13 are used by the drive subsystem and configured in TunerConstants */
        public static final int INTAKE = 20;
    }

    public static class Drive
    {
        
    }

    public static class Intake
    {
        public static final double INTAKE_VOLTS  = 8.0;
        public static final double REVERSE_VOLTS = -4.0;
        public static final int    CURRENT_LIMIT = 40;

        public static final int CAMERA_DEVICE_INDEX = 0;
        public static final String CAMERA_NAME = "IntakeCam";
        public static final int CAMERA_WIDTH = 320;
        public static final int CAMERA_HEIGHT = 240;
        public static final int CAMERA_FPS = 15;
    }

    public static class General
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MOTOR_VOLTAGE = 12.0;
    }
}
