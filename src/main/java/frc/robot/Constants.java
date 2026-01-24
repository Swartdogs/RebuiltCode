package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

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
        public static final int INTAKE       = 20;
        public static final int TURRET_MOTOR = 24;
    }

    public static class Drive
    {
        public static final double MAX_SPEED        = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
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

    public static class Shooter
    {
        // Turret
        public static final double       TURRET_CURRENT_LIMIT = 40.0;
        public static final double       TURRET_KP            = 2.4;   // TODO: Tune
        public static final double       TURRET_KI            = 0.0;
        public static final double       TURRET_KD            = 0.1;
        public static final double       TURRET_GEAR_RATIO    = 1.0;   // TODO: Measure
        public static final double       TURRET_MIN_ANGLE     = -90.0; // degrees
        public static final double       TURRET_MAX_ANGLE     = 90.0;  // degrees
        public static final double       TURRET_HOME_ANGLE    = 0.0;   // Forward-facing when no target
        public static final double       TURRET_TOLERANCE     = 2.0;   // degrees
        public static final String       LIMELIGHT_NAME       = "limelight-shooter";
        public static final List<Double> BLUE_HUB_TAG_IDS     = List.of(2.0, 3.0, 4.0, 5.0, 8.0, 9.0, 10.0, 11.0);
        public static final List<Double> RED_HUB_TAG_IDS      = List.of(18.0, 19.0, 20.0, 21.0, 24.0, 25.0, 26.0, 27.0);
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
        public static final double MAX_TILT_FOR_VISION_DEG = 10.0; // TODO: Find the correct value
    }
}
