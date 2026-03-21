package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.TunerConstants;
import edu.wpi.first.math.system.plant.DCMotor;

public final class Constants
{
    public static class LoggingConstants
    {
        /**
         * Mirrors NetworkTables data to a WPILib .wpilog file so AdvantageScope can
         * inspect match telemetry after the fact.
         */
        public static final boolean ENABLE_WPILIB_DATA_LOG  = true;
        /**
         * Phoenix SignalLogger writes CTRE .hoot logs. Leave this disabled unless we
         * intentionally want Phoenix replay/SysId logging.
         */
        public static final boolean ENABLE_CTRE_SIGNAL_LOG  = false;
        /**
         * Loads a .hoot file for Phoenix replay. This is separate from live logging so
         * we can keep replay support off by default without changing the path.
         */
        public static final boolean ENABLE_CTRE_HOOT_REPLAY = false;
        public static final String  CTRE_HOOT_REPLAY_LOG    = "./logs/example.hoot";
    }

    public static class CANConstants
    {
        /*
         * CAN IDs 1 through 13 are used by the drive subsystem and configured in
         * TunerConstants
         */
        public static final int INTAKE          = 14;  // Vortex
        public static final int INTAKE_EXTEND   = 15;  // Vortex
        public static final int FEEDER_MOTOR    = 16;  // Vortex
        public static final int TURRET_MOTOR    = 17;  // Talon
        public static final int FLYWHEEL_LEAD   = 18;  // Vortex
        public static final int FLYWHEEL_FOLLOW = 19;  // Vortex
        public static final int CLIMBER_EXTEND  = 21;
        public static final int CLIMBER_ROTATE  = 22;
        public static final int ROTOR_MOTOR     = 23; // Vortex
    }

    public static class AIOConstants
    {
        public static final int TURRET_POTENTIOMETER = 0; // TODO
    }

    public static class AutoConstants
    {
        // Dashboard display
        public static final String AUTO_MODE_CHOOSER_NAME  = "Auto Mode";
        public static final String AUTO_START_CHOOSER_NAME = "Auto Start Position";
        public static final String AUTO_DRIVE_CHOOSER_NAME = "Auto Drive Path";
        public static final String AUTO_DELAY_CHOOSER_NAME = "Auto Delay";

        // Auto driving
        public static final double DRIVE_KP  = 3.0;
        public static final double DRIVE_KD  = 0.1;
        public static final double ROTATE_KP = 0.0;
        public static final double ROTATE_KD = 0.0;
    }

    public static class DriveConstants
    {
        public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts.times(0.5); // kSpeedAt12Volts
        // desired top speed
        public static final AngularVelocity MAX_ANGULAR_RATE   = RotationsPerSecond.of(0.75); // 3/4 of a rotation per
                                                                                              // second max angular
                                                                                              // velocity
        public static final Dimensionless   TRANSLATE_DEADBAND = Percent.of(8);
        public static final Dimensionless   ROTATE_DEADBAND    = Percent.of(8);
        public static final Dimensionless   SLOW_MODE_SCALE    = Percent.of(35);
        public static final Dimensionless   FULL_SPEED_SCALE   = Percent.of(100);
    }

    public static class IntakeConstants
    {
        public static final Voltage                      INTAKE_VOLTS                  = Volts.of(9.0);
        public static final Voltage                      REVERSE_VOLTS                 = Volts.of(-9.0);
        public static final Current                      ROLLER_CURRENT_LIMIT_EXTENDED = Amps.of(80);
        public static final Current                      ROLLER_CURRENT_LIMIT_ACTIVE   = Amps.of(40);
        public static final Current                      EXTENSION_CURRENT_LIMIT       = Amps.of(60);
        public static final int                          CAMERA_DEVICE_INDEX           = 0;
        public static final String                       CAMERA_NAME                   = "IntakeCam";
        public static final int                          CAMERA_WIDTH                  = 320;
        public static final int                          CAMERA_HEIGHT                 = 240;
        public static final int                          CAMERA_FPS                    = 15;
        public static final Per<DistanceUnit, AngleUnit> EXTENSION_CONVERSION_FACTOR   = Inches.of(12).div(Rotations.of(6));
        public static final Distance                     EXTENSION_MAX_POSITION        = Inches.of(12.0);
        public static final Distance                     EXTENSION_MIN_POSITION        = Inches.of(0);
        public static final Voltage                      EXTEND_VOLTS                  = Volts.of(1.5);
        public static final Voltage                      RETRACT_VOLTS                 = Volts.of(-2.5);
        public static final Voltage                      JIGGLE_RETRACT_VOLTS          = Volts.of(-6.0);
        public static final Voltage                      JIGGLE_EXTEND_VOLTS           = Volts.of(2.5);
        public static final double                       JIGGLE_RETRACT_FRACTION       = 0.2;
        public static final double                       JIGGLE_RETRACT_STEP_FRACTION  = 0.3;
        public static final Distance                     JIGGLE_LIMIT_MARGIN           = Inches.of(0.0);
        public static final Time                         JIGGLE_MOVE_TIMEOUT           = Seconds.of(0.60);
        public static final Time                         JIGGLE_PAUSE_TIME             = Seconds.of(0.02);
    }

    public static class GeneralConstants
    {
        public static final Time     LOOP_PERIOD    = Milliseconds.of(20);
        public static final Voltage  MOTOR_VOLTAGE  = Volts.of(12.0);
        public static final Voltage  SENSOR_VOLTAGE = Volts.of(5.0);
        public static final DCMotor  WINDOW_MOTOR   = new DCMotor(GeneralConstants.MOTOR_VOLTAGE.in(Volts), 9.2, 16.3, 1.6, RPM.of(90).in(RadiansPerSecond), 1);
        public static final Distance FIELD_SIZE_X   = Inches.of(651.2);
        public static final Distance FIELD_SIZE_Y   = Inches.of(317.7);
    }

    public static class ShooterConstants
    {
        // Flywheel
        public static final double                                    FLYWHEEL_KP            = 0.00015; // TODO: Tune
        public static final double                                    FLYWHEEL_KI            = 0.0; // TODO: Tune
        public static final double                                    FLYWHEEL_KD            = 0.0005;
        public static final Voltage                                   FLYWHEEL_KS            = Volts.of(0.0); // TODO: Tune - static friction voltage
        public static final Per<VoltageUnit, AngularVelocityUnit>     FLYWHEEL_KV            = Volts.of(12.0).div(RPM.of(6784.0));
        public static final Per<VoltageUnit, AngularAccelerationUnit> FLYWHEEL_KA            = Volts.of(0).per(RotationsPerSecondPerSecond); // TODO: Tune - acceleration voltage
        public static final Dimensionless                             FLYWHEEL_TOLERANCE     = Percent.of(15);
        public static final Current                                   FLYWHEEL_CURRENT_LIMIT = Amps.of(60);
        public static final AngularVelocity                           MANUAL_SHOOT_RPM       = RPM.of(3500.0);
        public static final AngularVelocity                           PASS_FLYWHEEL_VELOCITY = RPM.of(3000.0); // TODO: Tune

        // Turret
        public static final Current         TURRET_CURRENT_LIMIT                   = Amps.of(60.0);
        public static final double          TURRET_KP                              = 0.19;
        public static final double          TURRET_KI                              = 0.01;
        public static final double          TURRET_KD                              = 0.0115;
        public static final Voltage         TURRET_STATIC_FF                       = Volts.of(0.22);
        public static final Angle           TURRET_STATIC_FF_ERROR_DEADBAND        = Degrees.of(0.75);
        public static final Voltage         TURRET_MAX_OUTPUT_STEP_PER_LOOP        = Volts.of(3.0);
        public static final Angle           TURRET_HARD_MIN_ANGLE                  = Degrees.of(-180.0);
        public static final Angle           TURRET_HARD_MAX_ANGLE                  = Degrees.of(180.0);
        public static final boolean         TURRET_SENSOR_INVERTED                 = true;
        public static final Angle           TURRET_POT_OFFSET                      = Degrees.of(-10.0);
        public static final Angle           TURRET_SOFT_MIN_ANGLE                  = Degrees.of(-160.0);
        public static final Angle           TURRET_SOFT_MAX_ANGLE                  = Degrees.of(160.0);
        public static final Angle           TURRET_HOME_ANGLE                      = Degrees.of(0.0);
        public static final Angle           TURRET_TOLERANCE                       = Degrees.of(1.25);
        public static final Angle           TURRET_LINED_UP_HOLD_TOLERANCE         = Degrees.of(2.0);
        public static final Angle           TURRET_TARGET_SETPOINT_DEADBAND        = Degrees.of(0.1);
        public static final Angle           TURRET_MAX_SETPOINT_STEP_PER_LOOP      = Degrees.of(4.0);
        public static final Angle           TURRET_TRACK_COMMAND_DEADBAND          = Degrees.of(0.5);
        public static final Angle           TURRET_TRACK_COMMAND_MAX_STEP_PER_LOOP = Degrees.of(4.0);
        public static final Angle           TURRET_TRACK_TX_DEADBAND               = Degrees.of(1.0);
        public static final boolean         TURRET_USE_VISION_TX_TRIM              = false;
        public static final Time            TURRET_VISION_TX_FILTER_WINDOW         = Milliseconds.of(100.0);
        public static final Angle           TURRET_VISION_TX_MAX_STEP_PER_LOOP     = Degrees.of(0.4);
        public static final Angle           TURRET_TRACK_BIAS                      = Degrees.zero(); // TODO: Tune static shot bias from log data
        public static final boolean         TURRET_USE_LINEAR_DRIFT_COMPENSATION   = true;
        public static final Distance        TURRET_DRIFT_LATERAL_BIAS              = Inches.of(11.060660171779821);
        public static final Distance        TURRET_DRIFT_LATERAL_SINE_AMPLITUDE    = Inches.of(7.1890180494515015);
        public static final Angle           TURRET_DRIFT_LATERAL_PHASE_OFFSET      = Degrees.of(-20.866427056070258);
        public static final Angle           TURRET_LINEAR_DRIFT_MAX_CORRECTION     = Degrees.of(15.0);
        public static final double          TURRET_LINEAR_DRIFT_CORRECTION_SIGN    = -1.0;
        public static final String          LIMELIGHT_NAME                         = "limelight-shooter";
        public static final Angle           TURRET_PASS_TARGET                     = Degrees.of(180.0); // TODO: Validate in driver practice
        public static final Distance        TURRET_CENTER_TAG_TO_HUB_CENTER        = Inches.of(23.5);
        public static final List<Integer>   BLUE_HUB_TAG_IDS                       = List.of(18, 20, 21, 26);
        public static final List<Integer>   RED_HUB_TAG_IDS                        = List.of(2, 4, 5, 10);
        public static final List<Integer>   ALL_HUB_TAG_IDS                        = Stream.concat(BLUE_HUB_TAG_IDS.stream(), RED_HUB_TAG_IDS.stream()).toList();
        public static final Translation3d   CENTER_TAG_TO_HUB_CENTER_OFFSET        = new Translation3d(Inches.of(-23.5), Inches.zero(), Inches.zero());
        public static final Angle           TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD  = Degrees.of(90); // Hub "zero" is 90 degrees
                                                                                                     // left of robot forward
        public static final Translation2d   BLUE_HUB                               = new Translation2d(Inches.of(182.1), Inches.of(158.85));
        public static final Translation2d   RED_HUB                                = new Translation2d(Inches.of(469.1), Inches.of(158.85));
        public static final Translation2d   ROBOT_TO_TURRET_PIVOT                  = new Translation2d(Inches.of(-0.5), Inches.of(5.75));
        public static final Translation2d   TURRET_PIVOT_TO_RELEASE                = new Translation2d(Inches.of(1.0), Inches.zero()); // Measured release point is ~1 in forward of the pivot (turret -90), not
                                                                                                                                       // lateral
        public static final Translation2d   ROBOT_TO_TURRET_RELEASE                = ROBOT_TO_TURRET_PIVOT.plus(TURRET_PIVOT_TO_RELEASE);
        public static final Translation2d   ROBOT_TO_TURRET_CAMERA                 = new Translation2d(Inches.zero(), Inches.of(8.0));
        public static final Distance        TURRET_LIMELIGHT_HEIGHT                = Inches.of(24.625); // 24 5/8"
        public static final Distance        HUB_TAG_HEIGHT                         = Inches.of(44.25);
        public static final Distance        TURRET_TO_HUB_HEIGHT_DELTA             = HUB_TAG_HEIGHT.minus(TURRET_LIMELIGHT_HEIGHT);
        public static final Angle           TURRET_LIMELIGHT_PITCH                 = Degrees.zero();
        public static final Time            SWM_RELEASE_PHASE_DELAY                = Seconds.of(0.03);
        public static final int             SWM_LOOKAHEAD_ITERATIONS               = 8;
        public static final LinearVelocity  SWM_ENABLE_TRANSLATIONAL_SPEED         = MetersPerSecond.of(0.15);
        public static final AngularVelocity SWM_ENABLE_ANGULAR_SPEED               = DegreesPerSecond.of(10.0);
        public static final LinearVelocity  SWM_FEED_MAX_TRANSLATIONAL_SPEED       = MetersPerSecond.of(1.5);
        public static final AngularVelocity SWM_FEED_MAX_ANGULAR_SPEED             = DegreesPerSecond.of(45.0);
        public static final Angle           SWM_FEED_TURRET_TOLERANCE              = Degrees.of(5.0);
        public static final Time            SWM_FEED_STABILITY_WINDOW              = Milliseconds.of(60.0);
        private static final double         SWM_MIN_TOF_DISTANCE_IN                = 64.0;
        private static final double         SWM_MAX_TOF_DISTANCE_IN                = 159.0;

        // @formatter:off
        private static final InterpolatingDoubleTreeMap FLYWHEEL_SPEED_TABLE = InterpolatingDoubleTreeMap.ofEntries
        (
            Map.entry(50.0, 2500.0),
            Map.entry(75.5, 2950.0),
            Map.entry(101.0, 3400.0),
            Map.entry(113.0, 3850.0),
            Map.entry(130.0, 3900.0),
            Map.entry(151.0, 4600.0),
            Map.entry(183.0, 5100.0),
            Map.entry(204.0, 5850.0)
        );

        // Keep TOF as its own lookup so the moving-shot solve matches the empirical
        // map-based pattern used by teams like 5000, 6328, and Eeshwar's writeup.
        private static final InterpolatingDoubleTreeMap SHOT_TOF_TABLE = InterpolatingDoubleTreeMap.ofEntries
        (
            Map.entry(64.0, 0.542),
            Map.entry(85.0, 1.06),
            Map.entry(87.0, 1.02),
            Map.entry(134.0, 1.22),
            Map.entry(159.0, 1.74)
        );
        // @formatter:on

        // Feeder
        public static final Current FEEDER_CURRENT_LIMIT    = Amps.of(60);
        public static final Voltage FEEDER_VOLTAGE          = Volts.of(6);
        public static final Current ROTOR_CURRENT_LIMIT     = Amps.of(60);
        public static final Voltage ROTOR_FAST_VOLTAGE      = Volts.of(3.0);
        public static final Voltage ROTOR_MID_VOLTAGE       = Volts.of(1.5);
        public static final Voltage ROTOR_RETRACTED_VOLTAGE = Volts.of(1.0);

        // TODO: Tune these values with testing!
        public static AngularVelocity getFlywheelSpeedForDistance(Distance distance)
        {
            return RPM.of(FLYWHEEL_SPEED_TABLE.get(distance.in(Inches)));
        }

        public static Time getShotTimeOfFlight(Distance distance)
        {
            return Seconds.of(SHOT_TOF_TABLE.get(distance.in(Inches)));
        }

        public static boolean isMovingShotDistanceValid(Distance distance)
        {
            var distanceInches = distance.in(Inches);
            return distanceInches >= SWM_MIN_TOF_DISTANCE_IN && distanceInches <= SWM_MAX_TOF_DISTANCE_IN;
        }
    }

    public static class VisionConstants
    {
        public static final String         LEFT_CAMERA_NAME    = "limelight-left";
        public static final String         RIGHT_CAMERA_NAME   = "limelight-right";
        public static final Distance       MAX_DETECTION_RANGE = Meters.of(6.0);
        public static final Distance       XY_STD_DEV          = Meters.of(0.7);
        public static final Angle          THETA_STD_DEV       = Degrees.of(999999); // Trust gyro for heading, not vision
        public static final Matrix<N3, N1> STD_DEVS            = VecBuilder.fill(XY_STD_DEV.in(Meters), XY_STD_DEV.in(Meters), THETA_STD_DEV.in(Degrees));

        // Camera translations
        public static final Translation3d LEFT_CAMERA_TRANSLATION  = new Translation3d(Inches.of(-10.67), Inches.of(-10.67), Inches.of(9.25));
        public static final Translation3d RIGHT_CAMERA_TRANSLATION = new Translation3d(Inches.of(-10.67), Inches.of(10.67), Inches.of(9.25));

        // Camera rotations
        public static final Rotation3d LEFT_CAMERA_ROTATION  = new Rotation3d(Degrees.of(0), Degrees.of(13), Degrees.of(135));
        public static final Rotation3d RIGHT_CAMERA_ROTATION = new Rotation3d(Degrees.of(0), Degrees.of(13), Degrees.of(-135));

        // Camera offsets
        public static final Pose3d LEFT_CAMERA_OFFSET  = new Pose3d(LEFT_CAMERA_TRANSLATION, LEFT_CAMERA_ROTATION);
        public static final Pose3d RIGHT_CAMERA_OFFSET = new Pose3d(RIGHT_CAMERA_TRANSLATION, RIGHT_CAMERA_ROTATION);

        // Reject vision updates when spinning faster than this (MegaTag2 guidance)
        public static final AngularVelocity MAX_ANGULAR_RATE_FOR_VISION = DegreesPerSecond.of(720.0);

        // Reject vision updates when robot is tilted more than this (on ramp)
        public static final Angle MAX_TILT_FOR_VISION = Degrees.of(10.0); // TODO: find the correct value
    }

    public static class ClimberConstants
    {
        public static final Angle    L1_ROTATION          = Degrees.of(39.0); // TODO
        public static final Angle    L3_ROTATION          = Degrees.of(180.0); // TODO
        public static final Distance EXTENSION_THRESHOLD  = Inches.of(0.0); // TODO
        public static final Distance RETRACTION_THRESHOLD = Inches.of(0.0); // TODO
        public static final Voltage  EXTEND_OUTPUT        = Volts.of(1.0); // TODO: duty cycle
        public static final Voltage  RETRACT_VOLTAGE      = Volts.of(-1.0); // TODO: will be -extendoutput
        public static final Voltage  ROTATE_OUTPUT        = Volts.of(1.0); // TODO: duty cycle
        public static final Angle    ROTATION_TOLERANCE   = Degrees.of(2.0); // TODO
    }

    public static class SimulationConstants
    {
        public static final Distance EXTENDED_DISTANCE  = Inches.of(10);
        public static final Distance RETRACTED_DISTANCE = Inches.of(0);
    }
}
