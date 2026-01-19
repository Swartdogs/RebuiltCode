package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

public final class Constants
{
    public static class CAN
    {
        /*
         * CAN IDs 1 through 13 are used by the drive subsystem and configured in
         * TunerConstants
         */
        public static final int INTAKE = 20;
    }

    public static class Drive
    {
        public static final double MAX_SPEED        = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final double DEADBAND         = 0.05;
    }

    public static class Intake
    {
        public static final double INTAKE_VOLTS        = 8.0;
        public static final double REVERSE_VOLTS       = -4.0;
        public static final int    CURRENT_LIMIT       = 40;
        public static final int    CAMERA_DEVICE_INDEX = 0;
        public static final String CAMERA_NAME         = "IntakeCam";
        public static final int    CAMERA_WIDTH        = 320;
        public static final int    CAMERA_HEIGHT       = 240;
        public static final int    CAMERA_FPS          = 15;
    }

    public static class General
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MOTOR_VOLTAGE    = 12.0;
    }

    public static class Vision
    {
        public static final String         LEFT_CAMERA_NAME    = "limelight-left";
        public static final String         RIGHT_CAMERA_NAME   = "limelight-right";
        public static final double         MAX_DETECTION_RANGE = 6.0;    // meters
        public static final double         XY_STD_DEV          = 0.7;    // meters
        public static final double         THETA_STD_DEV       = 9999.0; // Trust gyro for heading, not vision
        public static final Matrix<N3, N1> STD_DEVS            = VecBuilder.fill(XY_STD_DEV, XY_STD_DEV, THETA_STD_DEV);

        // Reject vision updates when spinning faster than this (MegaTag2 guidance)
        public static final double MAX_ANGULAR_RATE_FOR_VISION_DEG_PER_SEC = 720.0;

        // Reject vision updates when robot is tilted more than this (on ramp)
        public static final double MAX_TILT_FOR_VISION_DEG = 10.0; // TODO: find the correct value
    }
}
