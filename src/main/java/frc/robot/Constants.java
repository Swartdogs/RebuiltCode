package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

public final class Constants
{
    public static class CANConstants
    {
        /*
         * CAN IDs 1 through 13 are used by the drive subsystem and configured in
         * TunerConstants
         */
        public static final int INTAKE          = 20;
        public static final int FLYWHEEL_LEAD   = 21;
        public static final int FLYWHEEL_FOLLOW = 22;
        public static final int HOOD_MOTOR      = 23;
        public static final int TURRET_MOTOR    = 24;
        public static final int FEEDER_MOTOR    = 25; // TODO: Confirm CAN ID
    }

    public static class AIOConstants
    {
        public static final int HOOD_POTENTIOMETER = 1; // TODO: Confirm AIO port wiring
    }

    public static class DriveConstants
    {
        public static final double MAX_SPEED        = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final double DEADBAND         = 0.05;
    }

    public static class IntakeConstants
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

    public static class GeneralConstants
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MOTOR_VOLTAGE    = 12.0; // TODO: This should likely be Voltage type, but easier to make it a double for
                                                            // now.
    }

    public static class ShooterConstants
    {
        public static final double FLYWHEEL_KP            = 0.0001; // TODO: Tune
        public static final double FLYWHEEL_KI            = 0.0;    // TODO: Tune
        public static final double FLYWHEEL_KD            = 0.0;
        public static final double FLYWHEEL_KS            = 0.0;            // TODO: Tune - static friction voltage
        public static final double FLYWHEEL_KV            = 12.0 / 6784.0; // Volts / RPM
        public static final double FLYWHEEL_KA            = 0.0;            // TODO: Tune - acceleration voltage
        public static final double FLYWHEEL_TOLERANCE     = 0.15; // 15% tolerance for atSpeed()
        public static final int    FLYWHEEL_CURRENT_LIMIT = 60;
        public static final double PASS_FLYWHEEL_RPM      = 3000.0; // TODO: Tune

        // Hood (VictorSPX with analog potentiometer)
        public static final double HOOD_KP            = 0.016; // TODO: Tune
        public static final double HOOD_KI            = 0.001; // TODO: Tune
        public static final double HOOD_KD            = 0.0;   // TODO: Tune
        public static final double HOOD_MIN_ANGLE     = 0.0;   // TODO: Confirm min angle (degrees)
        public static final double HOOD_MAX_ANGLE     = 45.0;  // TODO: Confirm max angle (degrees)
        public static final double HOOD_SIM_MAX_SPEED = 45.0; // TODO: compute from motor free speed and hood gear ratio
        public static final Angle  HOOD_SHOOT_ANGLE   = Degrees.of(50); // TODO: Find the degree needed to shoot from.
        public static final Angle  HOOD_PASS_ANGLE    = Degrees.of(30); // TODO: Find the degree to pass from.
        public static final Angle  HOOD_TOLERANCE     = Degrees.of(2);

        // Turret
        public static final double                      TURRET_CURRENT_LIMIT = 40.0;
        public static final double                      TURRET_KP            = 2.4;   // TODO: Tune
        public static final double                      TURRET_KI            = 0.0;
        public static final double                      TURRET_KD            = 0.1;
        public static final double                      TURRET_GEAR_RATIO    = 1.0;   // TODO: Measure
        public static final double                      TURRET_MIN_ANGLE     = -180.0; // degrees (full 360° rotation)
        public static final double                      TURRET_MAX_ANGLE     = 180.0;  // degrees
        public static final double                      TURRET_HOME_ANGLE    = 0.0;   // Forward-facing when no target
        public static final double                      TURRET_TOLERANCE     = 2.0;   // degrees
        public static final String                      LIMELIGHT_NAME       = "limelight-shooter";
        public static final List<Double>                BLUE_HUB_TAG_IDS     = List.of(2.0, 3.0, 4.0, 5.0);
        public static final List<Double>                RED_HUB_TAG_IDS      = List.of(18.0, 19.0, 20.0, 21.0);
        private static final InterpolatingDoubleTreeMap FLYWHEEL_SPEED_TABLE = InterpolatingDoubleTreeMap
                .ofEntries(Map.entry(0.0, 3000.0), Map.entry(2.0, 3000.0), Map.entry(3.5, 3500.0), Map.entry(5.0, 4000.0), Map.entry(6.5, 4500.0), Map.entry(7.0, 5000.0));
        private static final InterpolatingDoubleTreeMap HOOD_ANGLE_TABLE     = InterpolatingDoubleTreeMap
                .ofEntries(Map.entry(0.0, 20.0), Map.entry(2.0, 15.0), Map.entry(3.5, 22.0), Map.entry(5.0, 30.0), Map.entry(6.5, 38.0), Map.entry(7.0, 45.0));

        // Feeder
        public static final int    FEEDER_CURRENT_LIMIT = 60;
        public static final double FEEDER_VOLTAGE       = 6;

        // TODO: Tune these values with testing!
        public static AngularVelocity getFlywheelSpeedForDistance(double meters)
        {
            return RPM.of(FLYWHEEL_SPEED_TABLE.get(meters));
        }

        public static double getHoodAngleForDistance(double meters)
        {
            return HOOD_ANGLE_TABLE.get(meters);
        }
    }

    public static class VisionConstants
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
