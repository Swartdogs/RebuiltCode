package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

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
        public static final int INTAKE          = 14; // Vortex
        public static final int INTAKE_EXTEND   = 15; // Vortex
        public static final int FEEDER_MOTOR    = 16; // Vortex
        public static final int TURRET_MOTOR    = 17; // Talon
        public static final int FLYWHEEL_LEAD   = 18; // Vortex
        public static final int FLYWHEEL_FOLLOW = 19; // Vortex
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
        public static final double ROTATE_KP = 2.0;
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
        public static final Voltage                      INTAKE_VOLTS                  = Volts.of(4.0);
        public static final Voltage                      REVERSE_VOLTS                 = Volts.of(-6.0);
        public static final Current                      ROLLER_CURRENT_LIMIT_EXTENDED = Amps.of(80);
        public static final Current                      ROLLER_CURRENT_LIMIT_ACTIVE   = Amps.of(60);
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
        public static final Time                         RETRACT_TIMEOUT               = Seconds.of(4.0);
    }

    public static class GeneralConstants
    {
        public static final Time          LOOP_PERIOD    = Milliseconds.of(20);
        public static final Voltage       MOTOR_VOLTAGE  = Volts.of(12.0);
        public static final Voltage       SENSOR_VOLTAGE = Volts.of(5.0);
        public static final DCMotor       WINDOW_MOTOR   = new DCMotor(GeneralConstants.MOTOR_VOLTAGE.in(Volts), 9.2, 16.3, 1.6, RPM.of(90).in(RadiansPerSecond), 1);
        public static final Distance      FIELD_SIZE_X   = Inches.of(651.2);
        public static final Distance      FIELD_SIZE_Y   = Inches.of(317.7);
        public static final Translation2d FIELD_CENTER   = new Translation2d(FIELD_SIZE_X.div(2), FIELD_SIZE_Y.div(2));
    }

    public static class VisionConstants
    {
        public static final String         LEFT_CAMERA_NAME    = "limelight-left";
        public static final String         RIGHT_CAMERA_NAME   = "limelight-right";
        public static final Distance       MAX_DETECTION_RANGE = Meters.of(6.0);
        public static final Distance       AUTO_XY_STD_DEV     = Meters.of(4);
        public static final Distance       TELEOP_XY_STD_DEV   = Meters.of(0.7);
        public static final Angle          THETA_STD_DEV       = Degrees.of(999999); // Trust gyro for heading, not vision
        public static final Matrix<N3, N1> AUTO_STD_DEVS       = VecBuilder.fill(AUTO_XY_STD_DEV.in(Meters), AUTO_XY_STD_DEV.in(Meters), THETA_STD_DEV.in(Degrees));
        public static final Matrix<N3, N1> TELEOP_STD_DEVS     = VecBuilder.fill(TELEOP_XY_STD_DEV.in(Meters), TELEOP_XY_STD_DEV.in(Meters), THETA_STD_DEV.in(Degrees));

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

        // Reject vision updates when no AprilTag is detected within this range
        public static final Distance MAX_TARGET_DISTANCE = Feet.of(10);

        // AprilTag constants
        public static final List<Integer> BLUE_HUB_TAG_IDS = List.of(18, 20, 21, 26);
        public static final List<Integer> RED_HUB_TAG_IDS  = List.of(2, 4, 5, 10);
        public static final Translation2d BLUE_HUB         = new Translation2d(Inches.of(182.1), Inches.of(158.85));
        public static final Translation2d RED_HUB          = new Translation2d(Inches.of(469.1), Inches.of(158.85));
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

    public static class ShooterConstants
    {
        // Rotor
        public static final Current                                   ROTOR_CURRENT_LIMIT = Amps.of(80);
        public static final Dimensionless                             ROTOR_GEAR_RATIO    = Value.of(36).div(Value.of(1)); // 36:1
        public static final Voltage                                   ROTOR_KS            = Volts.of(0.0);
        public static final Per<VoltageUnit, AngularVelocityUnit>     ROTOR_KV            = Volts.of(12.0).div(RPM.of(6000.0).div(ROTOR_GEAR_RATIO));
        public static final Per<VoltageUnit, AngularAccelerationUnit> ROTOR_KA            = Volts.of(0.0).per(RotationsPerSecondPerSecond);
        public static final Per<VoltageUnit, AngularVelocityUnit>     ROTOR_KP            = Volts.of(0.025).per(RPM);
        public static final Per<VoltageUnit, AngularAccelerationUnit> ROTOR_KD            = Volts.of(0.00005).per(RPM.per(Second));

        // Flywheel
        public static final Current                                   FLYWHEEL_CURRENT_LIMIT = Amps.of(80);
        public static final Voltage                                   FLYWHEEL_KS            = Volts.of(0.0);
        public static final Per<VoltageUnit, AngularVelocityUnit>     FLYWHEEL_KV            = Volts.of(0.0).div(RPM.of(0.0));
        public static final Per<VoltageUnit, AngularAccelerationUnit> FLYWHEEL_KA            = Volts.of(0.0).per(RotationsPerSecondPerSecond);
        public static final Per<VoltageUnit, AngularVelocityUnit>     FLYWHEEL_KP            = Volts.of(0.0).per(RPM);
        public static final Per<VoltageUnit, AngularAccelerationUnit> FLYWHEEL_KD            = Volts.of(0.0).per(RPM.per(Second));
        public static final AngularVelocity                           FLYWHEEL_TOLERANCE     = RPM.of(50);
    }
}
