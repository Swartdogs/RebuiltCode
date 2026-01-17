package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;

@Logged
public class Vision extends SubsystemBase
{
    private final CommandSwerveDrivetrain _drive;
    private final String                  _cameraName;

    @Logged
    private Pose2d _latestPose = new Pose2d();

    @Logged
    private boolean _hasPose = false;

    @Logged
    private int _tagCount = 0;

    @Logged
    private double _avgTagDistance = 0.0;

    private double _lastTimestamp = 0.0;

    public Vision(CommandSwerveDrivetrain drive, String cameraName)
    {
        _drive      = drive;
        _cameraName = cameraName;
    }

    @Override
    public void periodic()
    {
        if (!RobotBase.isReal())
        {
            return;
        }

        // Always set robot orientation for MegaTag2 (required every frame)
        Rotation2d robotYaw = _drive.getPigeon2().getRotation2d();
        LimelightHelpers.SetRobotOrientation(
            _cameraName,
            robotYaw.getDegrees(),
            0, 0, 0, 0, 0
        );

        // Skip vision fusion when spinning too fast (MegaTag2 guidance)
        double gyroRateDegPerSec = Math.abs(_drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        if (gyroRateDegPerSec > Constants.Vision.MAX_ANGULAR_RATE_FOR_VISION_DEG_PER_SEC)
        {
            return;
        }

        var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(_cameraName);

        if (poseEstimate != null
            && poseEstimate.tagCount > 0
            && poseEstimate.avgTagDist < Constants.Vision.MAX_DETECTION_RANGE
            && poseEstimate.timestampSeconds != _lastTimestamp)
        {
            _latestPose     = poseEstimate.pose;
            _hasPose        = true;
            _tagCount       = poseEstimate.tagCount;
            _avgTagDistance = poseEstimate.avgTagDist;

            Matrix<N3, N1> stdDevs = calculateStdDevs(_tagCount, _avgTagDistance);
            _drive.addVisionMeasurement(_latestPose, poseEstimate.timestampSeconds, stdDevs);
            _lastTimestamp = poseEstimate.timestampSeconds;
        }
        else
        {
            _hasPose = false;
        }
    }

    private Matrix<N3, N1> calculateStdDevs(int tagCount, double avgTagDistance)
    {
        double xyStdDev    = Constants.Vision.BASE_XY_STD_DEV;
        double thetaStdDev = Constants.Vision.BASE_THETA_STD_DEV;

        if (tagCount >= 2)
        {
            xyStdDev    *= Constants.Vision.MULTI_TAG_MULTIPLIER;
            thetaStdDev *= Constants.Vision.MULTI_TAG_MULTIPLIER;
        }

        if (avgTagDistance > Constants.Vision.FAR_TAG_THRESHOLD)
        {
            xyStdDev    *= Constants.Vision.FAR_TAG_MULTIPLIER;
            thetaStdDev *= Constants.Vision.FAR_TAG_MULTIPLIER;
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
}
