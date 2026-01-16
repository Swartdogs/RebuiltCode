package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class Vision extends SubsystemBase
{
    private final VisionIO                  _io;
    private final VisionIO.VisionIOInputs   _inputs        = new VisionIO.VisionIOInputs();
    private final CommandSwerveDrivetrain   _drive;
    private final String                    _cameraName;

    private double _lastTimestamp = 0.0;

    public Vision(CommandSwerveDrivetrain drive, VisionIO io, String cameraName)
    {
        _drive      = drive;
        _io         = io;
        _cameraName = cameraName;
    }

    public static Vision create(CommandSwerveDrivetrain drive, String cameraName)
    {
        VisionIO io = switch (Constants.CURRENT_MODE)
        {
            case REAL -> new VisionIOLimelight(cameraName, () -> getRobotRotation(drive));
            case SIM  -> new VisionIOSim();
            default   -> new VisionIO() {};
        };
        return new Vision(drive, io, cameraName);
    }

    private static Rotation2d getRobotRotation(CommandSwerveDrivetrain drive)
    {
        // Use raw gyro yaw, not fused pose rotation, to avoid feedback loop
        return drive.getPigeon2().getRotation2d();
    }

    @Override
    public void periodic()
    {
        // Always update inputs so SetRobotOrientation runs every frame
        _io.updateInputs(_inputs);

        // Skip vision fusion when spinning too fast (MegaTag2 guidance)
        double gyroRateDegPerSec = Math.abs(_drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        if (gyroRateDegPerSec > Constants.Vision.MAX_ANGULAR_RATE_FOR_VISION_DEG_PER_SEC)
        {
            return;
        }

        if (_inputs.hasPose && _inputs.captureTimestamp != _lastTimestamp)
        {
            Matrix<N3, N1> stdDevs = calculateStdDevs(_inputs.tagCount, _inputs.avgTagDistance);
            _drive.addVisionMeasurement(_inputs.pose, _inputs.captureTimestamp, stdDevs);
            _lastTimestamp = _inputs.captureTimestamp;
        }
    }

    private Matrix<N3, N1> calculateStdDevs(int tagCount, double avgTagDistance)
    {
        double xyStdDev    = Constants.Vision.BASE_XY_STD_DEV;
        double thetaStdDev = Constants.Vision.BASE_THETA_STD_DEV;

        // Trust multi-tag estimates more
        if (tagCount >= 2)
        {
            xyStdDev    *= Constants.Vision.MULTI_TAG_MULTIPLIER;
            thetaStdDev *= Constants.Vision.MULTI_TAG_MULTIPLIER;
        }

        // Trust far tags less
        if (avgTagDistance > Constants.Vision.FAR_TAG_THRESHOLD)
        {
            xyStdDev    *= Constants.Vision.FAR_TAG_MULTIPLIER;
            thetaStdDev *= Constants.Vision.FAR_TAG_MULTIPLIER;
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    public String getCameraName()
    {
        return _cameraName;
    }

    public boolean hasPose()
    {
        return _inputs.hasPose;
    }

    public int getTagCount()
    {
        return _inputs.tagCount;
    }
}
