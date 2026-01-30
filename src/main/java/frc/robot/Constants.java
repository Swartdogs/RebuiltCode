package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public final class Constants
{
    public static class CAN
    {
        /*
         * CAN IDs 1 through 13 are used by the drive subsystem and configured in
         * TunerConstants
         */
        public static final int INTAKE          = 20;
        public static final int FLYWHEEL_LEAD   = 21;
        public static final int FLYWHEEL_FOLLOW = 22;
    }

    public static class Drive
    {
        public static final LinearVelocity  MAX_SPEED        = TunerConstants.kSpeedAt12Volts.div(Value.of(2)); // kSpeedAt12Volts desired top speed
        public static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75); // 3/4 of a rotation per second max angular velocity
        public static final Dimensionless   DEADBAND         = Percent.of(5);
    }

    public static class Intake
    {
        public static final Voltage INTAKE_VOLTS        = Volts.of(8.0);
        public static final Voltage REVERSE_VOLTS       = Volts.of(-4.0);
        public static final Current CURRENT_LIMIT       = Amps.of(40);
        public static final int    CAMERA_DEVICE_INDEX = 0;
        public static final String CAMERA_NAME         = "IntakeCam";
        public static final int    CAMERA_WIDTH        = 320;
        public static final int    CAMERA_HEIGHT       = 240;
        public static final int    CAMERA_FPS          = 15;
    }

    public static class General
    {
        public static final Time    LOOP_PERIOD = Milliseconds.of(20);
        public static final Voltage MOTOR_VOLTAGE    = Volts.of(12.0);
    }

    public static class Shooter
    {
        public static final double FLYWHEEL_KP            = 0.0001; // TODO: Tune
        public static final double FLYWHEEL_KD            = 0.0;
        public static final double FLYWHEEL_KS            = 0.0;            // TODO: Tune - static friction voltage
        public static final double FLYWHEEL_KV            = 12.0 / 6784.0;  // Volts / NEO Vortex free speed RPM
        public static final double FLYWHEEL_KA            = 0.0;            // TODO: Tune - acceleration voltage
        public static final Dimensionless FLYWHEEL_TOLERANCE     = Percent.of(15); // 15% tolerance for atSpeed()
        public static final Current       FLYWHEEL_CURRENT_LIMIT = Amps.of(60);
    }

    public static class Vision
    {
        public static final String         LEFT_CAMERA_NAME    = "limelight-left";
        public static final String         RIGHT_CAMERA_NAME   = "limelight-right";
        public static final Distance       MAX_DETECTION_RANGE = Meters.of(6.0);
        public static final Distance       XY_STD_DEV          = Meters.of(0.7);
        public static final Angle          THETA_STD_DEV       = Degrees.of(9999.0); // Trust gyro for heading, not vision
        public static final Matrix<N3, N1> STD_DEVS            = VecBuilder.fill(XY_STD_DEV.in(Meters), XY_STD_DEV.in(Meters), THETA_STD_DEV.in(Degrees));

        // Reject vision updates when spinning faster than this (MegaTag2 guidance)
        public static final AngularVelocity MAX_ANGULAR_RATE_FOR_VISION = DegreesPerSecond.of(720.0);

        // Reject vision updates when robot is tilted more than this (on ramp)
        public static final Angle MAX_TILT_FOR_VISION = Degrees.of(10.0); // TODO: find the correct value
    }
}
