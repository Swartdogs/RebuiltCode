package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
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
        public static final int HOOD_MOTOR      = 23;
        public static final int TURRET_MOTOR    = 24;
        public static final int INTAKE_EXTENSION = 21;
    }

    public static class AIOConstants
    {
        public static final int HOOD_POTENTIOMETER = 0; // TODO: Confirm AIO port wiring
    }

    public static class DriveConstants
    {
        public static final double MAX_SPEED        = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final double DEADBAND         = 0.05;
    }

    public static class IntakeConstants
    {
        public static final Voltage  INTAKE_VOLTS                = Volts.of(8.0);
        public static final Voltage  REVERSE_VOLTS               = Volts.of(-4.0);
        public static final int      CURRENT_LIMIT               = 40;
        public static final int      CAMERA_DEVICE_INDEX         = 0;
        public static final String   CAMERA_NAME                 = "IntakeCam";
        public static final int      CAMERA_WIDTH                = 320;
        public static final int      CAMERA_HEIGHT               = 240;
        public static final int      CAMERA_FPS                  = 15;
        public static final double   EXTENSION_CONVERSION_FACTOR = 0;
        public static final Distance EXTENSION_MAX_POSITION      = Inches.of(0.0);
        public static final Distance EXTENSION_MIN_POSITION      = Inches.of(0.0);
        public static final Voltage  EXTEND_OUTPUT               = Volts.zero();
        public static final Voltage  RETRACT_VOLTS               = Volts.zero();
    }

    public static class GeneralConstants
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MOTOR_VOLTAGE    = 12.0;
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
}
