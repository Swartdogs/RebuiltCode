package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.system.plant.DCMotor;
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
        public static final int INTAKE           = 20;
        public static final int FLYWHEEL_LEAD    = 21;
        public static final int FLYWHEEL_FOLLOW  = 22;
        public static final int CLIMBER_EXTEND   = 26;
        public static final int INTAKE_EXTEND    = 27;
        public static final int CLIMBER_ROTATE   = 28;
        public static final int INTAKE_EXTENSION = 0;
        public static final int HOOD_MOTOR       = 23;
        public static final int TURRET_MOTOR     = 24;
        public static final int FEEDER_MOTOR     = 25; // TODO: Confirm CAN ID
    }

    public static class AIOConstants
    {
        public static final int TURRET_POTENTIOMETER = 0; // TODO
    }

    public static class DIOConstants
    {
        public static final int HOOD_ENCODER = 1; // TODO: Confirm AIO port wiring
    }

    public static class DriveConstants
    {
        public static final double MAX_SPEED        = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final double DEADBAND         = 0.05;
    }

    public static class IntakeConstants
    {
        public static final Voltage INTAKE_VOLTS                = Volts.of(8.0);
        public static final Voltage REVERSE_VOLTS               = Volts.of(-4.0);
        public static final int     CURRENT_LIMIT               = 40;
        public static final int     CAMERA_DEVICE_INDEX         = 0;
        public static final String  CAMERA_NAME                 = "IntakeCam";
        public static final int     CAMERA_WIDTH                = 320;
        public static final int     CAMERA_HEIGHT               = 240;
        public static final int     CAMERA_FPS                  = 15;
        public static final double  EXTENSION_CONVERSION_FACTOR = 1;
        public static final double  EXTENSION_MAX_POSITION      = 10.;
        public static final double  EXTENSION_MIN_POSITION      = 0.;
        public static final Voltage EXTEND_VOLTS                = Volts.of(6.7);
        public static final Voltage RETRACT_VOLTS               = Volts.of(-6.9);
    }

    public static class GeneralConstants
    {
        public static final double  LOOP_PERIOD_SECS = 0.02;
        public static final double  MOTOR_VOLTAGE    = 12.0;
        public static final double  SENSOR_VOLTAGE   = 5.0;
        public static final DCMotor WINDOW_MOTOR     = new DCMotor(GeneralConstants.MOTOR_VOLTAGE, 9.2, 16.3, 1.6, RPM.of(90).in(RadiansPerSecond), 1);
    }

    public static class ShooterConstants
    {
        // Flywheel
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
        public static final int    HOOD_ANALOG_INPUT   = 0;     // AIO port for potentiometer
        public static final double HOOD_KP             = 0.016; // From Sidewinder
        public static final double HOOD_KI             = 0.001; // From Sidewinder
        public static final double HOOD_KD             = 0.0;   // TODO: Tune
        public static final double HOOD_MIN_ANGLE      = 0.0;   // TODO: Confirm min angle (degrees)
        public static final double HOOD_MAX_ANGLE      = 90.0;  // TODO: Confirm max angle (degrees)
        public static final double HOOD_GEAR_RATIO     = 1.0 / 12.0;
        public static final Angle  HOOD_SHOOT_ANGLE    = Degrees.of(90); // TODO: Find the degree needed to shoot from.
        public static final Angle  HOOD_PASS_ANGLE     = Degrees.of(78); // TODO: Find the degree to pass from.
        public static final Angle  HOOD_TOLERANCE      = Degrees.of(2);
        public static final double HOOD_RAW_MIN        = 1035;  // TODO: Calibrate - analog value at min angle
        public static final double HOOD_RAW_MAX        = 335;   // TODO: Calibrate - analog value at max angle
        public static final double PASS_HOOD_ANGLE_DEG = 20.0;   // TODO: Tune
        public static final double HOOD_SIM_MAX_SPEED  = 45.0; // TODO: compute from motor free speed and hood gear ratio

        // Turret
        public static final double                      TURRET_CURRENT_LIMIT = 40.0;
        public static final double                      TURRET_KP            = 2.4;   // TODO: Tune
        public static final double                      TURRET_KI            = 0.0;
        public static final double                      TURRET_KD            = 0.1;
        public static final double                      TURRET_GEAR_RATIO    = 13.0;
        public static final double                      TURRET_MIN_ANGLE     = -180.0; // degrees (full 360° rotation)
        public static final double                      TURRET_MAX_ANGLE     = 180.0;  // degrees
        public static final double                      TURRET_HOME_ANGLE    = 0.0;   // Forward-facing when no target
        public static final double                      TURRET_TOLERANCE     = 2.0;   // degrees
        public static final int                         BLUE_CENTER_TAG_ID   = 26;
        public static final int                         BLUE_LEFT_TAG_ID     = 25;
        public static final int                         RED_CENTER_TAG_ID    = 10;
        public static final int                         RED_LEFT_TAG_ID      = 9;
        public static final double                      TURRET_CL_METERS     = 0.3556; // 14.00 in from GE-26300
                                                                                       // (https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf)
        public static final double                      TURRET_CH_METERS     = 0.5969; // 23.50 in from GE-26300
        public static final String                      LIMELIGHT_NAME       = "limelight-shooter";
        public static final Angle                       TURRET_PASS_TARGET   = Degrees.of(180.0); // TODO: Validate in driver practice
        private static final InterpolatingDoubleTreeMap FLYWHEEL_SPEED_TABLE = InterpolatingDoubleTreeMap
                .ofEntries(Map.entry(0.0, 3000.0), Map.entry(2.0, 3000.0), Map.entry(3.5, 3500.0), Map.entry(5.0, 4000.0), Map.entry(6.5, 4500.0), Map.entry(7.0, 5000.0));
        private static final InterpolatingDoubleTreeMap HOOD_ANGLE_TABLE     = InterpolatingDoubleTreeMap
                .ofEntries(Map.entry(0.0, 20.0), Map.entry(2.0, 15.0), Map.entry(3.5, 22.0), Map.entry(5.0, 30.0), Map.entry(6.5, 38.0), Map.entry(7.0, 45.0));
        public static final List<Integer>               BLUE_HUB_TAG_IDS     = List.of(2, 3, 4, 5, 8, 9, 10, 11);
        public static final List<Integer>               RED_HUB_TAG_IDS      = List.of(18, 19, 20, 21, 24, 25, 26, 27);

        // Feeder
        public static final int    FEEDER_CURRENT_LIMIT = 60;
        public static final double FEEDER_VOLTAGE       = 6;

        // TODO: Tune these values with testing!
        public static AngularVelocity getFlywheelSpeedForDistance(double meters)
        {
            return RPM.of(FLYWHEEL_SPEED_TABLE.get(meters));
        }

        // TODO: Tune these values with testing!

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

        // Camera translations
        public static final Translation3d LEFT_CAMERA_TRANSLATION  = new Translation3d(Inches.of(0.875), Inches.of(13), Inches.of(7.625));
        public static final Translation3d RIGHT_CAMERA_TRANSLATION = new Translation3d(Inches.of(15.25), Inches.of(5.75), Inches.of(7.5));

        // Camera rotations
        public static final Rotation3d LEFT_CAMERA_ROTATION  = new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-90));
        public static final Rotation3d RIGHT_CAMERA_ROTATION = new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0));

        // Camera offsets
        public static final Pose3d LEFT_CAMERA_OFFSET  = new Pose3d(LEFT_CAMERA_TRANSLATION, LEFT_CAMERA_ROTATION);
        public static final Pose3d RIGHT_CAMERA_OFFSET = new Pose3d(RIGHT_CAMERA_TRANSLATION, RIGHT_CAMERA_ROTATION);

        // Reject vision updates when spinning faster than this (MegaTag2 guidance)
        public static final double MAX_ANGULAR_RATE_FOR_VISION_DEG_PER_SEC = 720.0;

        // Reject vision updates when robot is tilted more than this (on ramp)
        public static final double MAX_TILT_FOR_VISION_DEG = 10.0; // TODO: find the correct value
    }

    public static class ClimberConstants
    {
        public static final double  L1_ROTATION          = 39.0; // TODO
        public static final double  L3_ROTATION          = 180.0; // TODO
        public static final double  EXTENSION_THRESHOLD  = 0.0; // TODO
        public static final double  RETRACTION_THRESHOLD = 0.0; // TODO
        public static final Voltage EXTEND_OUTPUT        = Volts.of(1.0); // TODO: duty cycle
        public static final Voltage RETRACT_VOLTAGE      = Volts.of(-1.0); // TODO: will be -extendoutput
        public static final double  ROTATE_OUTPUT        = 1.0; // TODO: duty cycle
        public static final double  ROTATION_TOLERANCE   = 2.0; // TODO
    }

    public static class SimulationConstants
    {
        public static final Distance EXTENDED_DISTANCE  = Inches.of(10);
        public static final Distance RETRACTED_DISTANCE = Inches.of(0);
    }
}
