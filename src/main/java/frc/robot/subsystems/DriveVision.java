package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.Constants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

@Logged
public class DriveVision {
    private final CommandSwerveDrivetrain _drivetrain;

    private final Limelight _limelightLeft;
    private final Limelight _limelightRight;

    private final LimelightPoseEstimator _poseEstimatorLeft;
    private final LimelightPoseEstimator _poseEstimatorRight;

    private double _lastTimestampLeft = 0.0;
    private double _lastTimestampRight = 0.0;

    @Logged
    private boolean _hasVisionLeft = false;

    @Logged
    private boolean _hasVisionRight = false;

    public DriveVision(CommandSwerveDrivetrain drivetrain) {
        _drivetrain = drivetrain;

        if (RobotBase.isReal()) {
            _limelightLeft = new Limelight(Constants.Vision.LEFT_CAMERA_NAME);
            _limelightRight = new Limelight(Constants.Vision.RIGHT_CAMERA_NAME);

            _poseEstimatorLeft = _limelightLeft.createPoseEstimator(EstimationMode.MEGATAG2);
            _poseEstimatorRight = _limelightRight.createPoseEstimator(EstimationMode.MEGATAG2);
        } else {
            _limelightLeft = null;
            _limelightRight = null;
            _poseEstimatorLeft = null;
            _poseEstimatorRight = null;
        }
    }

    public void update() {
        if (_limelightLeft == null) {
            return;
        }

        if (isRotatingTooFast() || isOnBump()) {
            _hasVisionLeft = false;
            _hasVisionRight = false;
            return;
        }

        // TODO: Learn how Limelight MegaTag2 works internally and consider implementing
        // our own pose prediction using the raw fiducial data (tag positions, distances,
        // ambiguity values) instead of relying on the pre-computed pose estimate.

        var rotation = _drivetrain.getPigeon2().getRotation3d();
        var angularVelocity = new AngularVelocity3d(
            DegreesPerSecond.of(_drivetrain.getPigeon2().getAngularVelocityXWorld().getValueAsDouble()),
            DegreesPerSecond.of(_drivetrain.getPigeon2().getAngularVelocityYWorld().getValueAsDouble()),
            DegreesPerSecond.of(_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())
        );
        var orientation = new Orientation3d(rotation, angularVelocity);

        _limelightLeft.getSettings().withRobotOrientation(orientation).save();
        _limelightRight.getSettings().withRobotOrientation(orientation).save();

        _hasVisionLeft = processLimelight(_poseEstimatorLeft, _lastTimestampLeft);
        if (_hasVisionLeft) {
            _lastTimestampLeft = _poseEstimatorLeft.getPoseEstimate()
                .map(e -> e.timestampSeconds)
                .orElse(_lastTimestampLeft);
        }

        _hasVisionRight = processLimelight(_poseEstimatorRight, _lastTimestampRight);
        if (_hasVisionRight) {
            _lastTimestampRight = _poseEstimatorRight.getPoseEstimate()
                .map(e -> e.timestampSeconds)
                .orElse(_lastTimestampRight);
        }
    }

    private boolean processLimelight(LimelightPoseEstimator poseEstimator, double lastTimestamp) {
        Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();

        if (estimate.isEmpty()) {
            return false;
        }

        PoseEstimate poseEstimate = estimate.get();

        if (poseEstimate.tagCount == 0
            || poseEstimate.avgTagDist > Constants.Vision.MAX_DETECTION_RANGE
            || poseEstimate.timestampSeconds == lastTimestamp) {
            return false;
        }

        Matrix<N3, N1> stdDevs = VecBuilder.fill(
            Constants.Vision.XY_STD_DEV,
            Constants.Vision.XY_STD_DEV,
            Constants.Vision.THETA_STD_DEV
        );
        _drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds, stdDevs);

        return true;
    }

    private boolean isRotatingTooFast() {
        double gyroRateDegPerSec = Math.abs(_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        return gyroRateDegPerSec > Constants.Vision.MAX_ANGULAR_RATE_FOR_VISION_DEG_PER_SEC;
    }

    private boolean isOnBump() {
        var rotation = _drivetrain.getPigeon2().getRotation3d();
        double pitchDeg = Math.abs(Math.toDegrees(rotation.getY()));
        double rollDeg = Math.abs(Math.toDegrees(rotation.getX()));
        return pitchDeg > Constants.Vision.MAX_TILT_FOR_VISION_DEG
            || rollDeg > Constants.Vision.MAX_TILT_FOR_VISION_DEG;
    }
}
