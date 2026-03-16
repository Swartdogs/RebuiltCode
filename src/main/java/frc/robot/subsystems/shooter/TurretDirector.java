package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Utilities;
import limelight.Limelight;
import limelight.networktables.target.AprilTagFiducial;

@Logged
public class TurretDirector
{
    public enum ShotMode
    {
        Idle, Track, Pass
    }

    public record ShotSolution(boolean valid, Angle turretAngle, Distance distance, ShotMode mode, Pose2d targetPose, boolean hasVisionTarget)
    {
    }

    private record MovingTrackState(Pose2d releasePose, Translation2d lookaheadTurretOrigin, Distance lookaheadDistance, Time timeOfFlight)
    {
    }

    private final Supplier<SwerveDriveState> _swerveStateSupplier;
    private final Limelight                  _limelight;
    private List<Integer>                    _targetTagFilter;
    private ShotSolution                     _currentSolution;
    @Logged
    private Distance                         _targetDistance;
    @Logged
    private Pose2d                           _targetPose;
    @Logged
    private boolean                          _limelightHasTarget;
    @Logged
    private boolean                          _hubActive;
    @Logged
    private boolean                          _targetValid;
    @Logged
    private Pose2d                           _predictedReleasePose;
    @Logged
    private Pose2d                           _lookaheadTurretOriginPose;
    @Logged
    private Time                             _shotTimeOfFlight;

    public TurretDirector(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        _swerveStateSupplier       = swerveStateSupplier;
        _limelight                 = new Limelight(ShooterConstants.LIMELIGHT_NAME);
        _targetTagFilter           = List.of();
        _currentSolution           = new ShotSolution(false, ShooterConstants.TURRET_HOME_ANGLE, Meters.zero(), ShotMode.Idle, new Pose2d(), false);
        _targetDistance            = Meters.zero();
        _targetPose                = new Pose2d();
        _limelightHasTarget        = false;
        _hubActive                 = false;
        _targetValid               = false;
        _predictedReleasePose      = new Pose2d();
        _lookaheadTurretOriginPose = new Pose2d();
        _shotTimeOfFlight          = Seconds.zero();

        _limelight.getSettings().withAprilTagOffset(ShooterConstants.CENTER_TAG_TO_HUB_CENTER_OFFSET).save();
        updateFilter(Utilities.getOurHubTagIds(), true);
    }

    public ShotSolution calculate(ShotMode shotMode)
    {
        return calculate(shotMode, false);
    }

    public ShotSolution calculate(ShotMode shotMode, boolean useMovingShotMath)
    {
        var swerveState    = _swerveStateSupplier.get();
        var targetTagIds   = Utilities.getOurHubTagIds();
        var isBlueAlliance = Utilities.isBlueAlliance();

        updateFilter(targetTagIds, false);

        _hubActive = Utilities.isHubActive();

        var fiducials = new AprilTagFiducial[0];
        var results   = _limelight.getLatestResults();

        if (results.isPresent())
        {
            fiducials = results.get().targets_Fiducials;
        }

        var solution = calculate(shotMode, useMovingShotMath, swerveState, Utilities.getHubCoordinates(), targetTagIds, isBlueAlliance, _hubActive, fiducials);
        _currentSolution    = solution;
        _targetDistance     = solution.distance();
        _targetPose         = solution.targetPose();
        _limelightHasTarget = solution.hasVisionTarget();
        _targetValid        = solution.valid();
        return solution;
    }

    ShotSolution calculate(ShotMode shotMode, SwerveDriveState swerveState, Translation2d hubCoordinates, List<Integer> targetTagIds, boolean isBlueAlliance, boolean hubActive, AprilTagFiducial... fiducials)
    {
        return calculate(shotMode, false, swerveState, hubCoordinates, targetTagIds, isBlueAlliance, hubActive, fiducials);
    }

    ShotSolution calculate(ShotMode shotMode, boolean useMovingShotMath, SwerveDriveState swerveState, Translation2d hubCoordinates, List<Integer> targetTagIds, boolean isBlueAlliance, boolean hubActive, AprilTagFiducial... fiducials)
    {
        return switch (shotMode)
        {
            case Track -> useMovingShotMath ? calculateMovingTrack(swerveState, hubCoordinates, targetTagIds, hubActive, fiducials) : calculateTrack(swerveState, hubCoordinates, targetTagIds, hubActive, fiducials);
            case Pass -> calculatePass(swerveState, isBlueAlliance);
            case Idle -> createIdleSolution(swerveState.Pose);
        };
    }

    private ShotSolution calculateTrack(SwerveDriveState swerveState, Translation2d hubCoordinates, List<Integer> targetTagIds, boolean hubActive, AprilTagFiducial... fiducials)
    {
        // Keep the named geometry points here, but preserve the old static shot
        // behavior for now: distance is still pivot-to-hub, and the release point
        // only contributes its lateral component as the legacy angle trim.
        var hubTranslationInRobotFrame = getHubTranslationInRobotFrame(swerveState, hubCoordinates);
        var pivotToHubTranslation      = hubTranslationInRobotFrame.minus(ShooterConstants.ROBOT_TO_TURRET_PIVOT);
        var distance                   = Meters.of(pivotToHubTranslation.getNorm());
        var rawSetpoint                = getTurretSetpointForRobotFrameTarget(pivotToHubTranslation);
        var bestTag                    = selectBestTag(targetTagIds, fiducials);

        if (bestTag != null)
        {
            var txCorrection = Degrees.of(-bestTag.tx);

            if (!MathUtil.isNear(0.0, txCorrection.in(Degrees), ShooterConstants.TURRET_TRACK_TX_DEADBAND.in(Degrees)))
            {
                rawSetpoint = rawSetpoint.minus(txCorrection);
            }
        }

        rawSetpoint = rawSetpoint.plus(getLegacyReleaseLateralCorrection(rawSetpoint, distance));

        logShotKinematics(swerveState.Pose, getPointInField(swerveState.Pose, ShooterConstants.ROBOT_TO_TURRET_PIVOT), distance, ShooterConstants.getShotTimeOfFlight(distance));

        return new ShotSolution(hubActive, rawSetpoint, distance, ShotMode.Track, buildTargetPose(swerveState.Pose, distance, rawSetpoint), bestTag != null);
    }

    private ShotSolution calculatePass(SwerveDriveState swerveState, boolean isBlueAlliance)
    {
        Angle    fieldTargetAngle;
        Distance distance;

        if (isBlueAlliance)
        {
            distance         = swerveState.Pose.getMeasureX();
            fieldTargetAngle = Degrees.of(180);
        }
        else
        {
            distance         = GeneralConstants.FIELD_SIZE_X.minus(swerveState.Pose.getMeasureX());
            fieldTargetAngle = Degrees.zero();
        }

        var rawSetpoint = fieldTargetAngle.minus(swerveState.Pose.getRotation().getMeasure()).minus(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD);

        logShotKinematics(swerveState.Pose, getPointInField(swerveState.Pose, ShooterConstants.ROBOT_TO_TURRET_PIVOT), distance, ShooterConstants.getShotTimeOfFlight(distance));

        return new ShotSolution(true, rawSetpoint, distance, ShotMode.Pass, buildTargetPose(swerveState.Pose, distance, rawSetpoint), false);
    }

    private ShotSolution calculateMovingTrack(SwerveDriveState swerveState, Translation2d hubCoordinates, List<Integer> targetTagIds, boolean hubActive, AprilTagFiducial... fiducials)
    {
        var movingTrackState = calculateMovingTrackState(swerveState, hubCoordinates);
        var lookaheadToHub   = hubCoordinates.minus(movingTrackState.lookaheadTurretOrigin());
        var rawSetpoint      = getTurretSetpointForFieldTarget(movingTrackState.releasePose(), lookaheadToHub.getAngle().getMeasure());
        var bestTag          = selectBestTag(targetTagIds, fiducials);

        rawSetpoint = rawSetpoint.plus(getLegacyReleaseLateralCorrection(rawSetpoint, movingTrackState.lookaheadDistance()));

        logShotKinematics(movingTrackState.releasePose(), movingTrackState.lookaheadTurretOrigin(), movingTrackState.lookaheadDistance(), movingTrackState.timeOfFlight());

        return new ShotSolution(
                hubActive && ShooterConstants.isMovingShotDistanceValid(movingTrackState.lookaheadDistance()), rawSetpoint, movingTrackState.lookaheadDistance(), ShotMode.Track,
                buildTargetPose(movingTrackState.releasePose(), movingTrackState.lookaheadDistance(), rawSetpoint), bestTag != null
        );
    }

    private Pose2d buildTargetPose(Pose2d robotPose, Distance distance, Angle rawSetpoint)
    {
        var forwardTranslation    = new Translation2d(distance, Meters.zero());
        var rotatedByTurretOffset = forwardTranslation.rotateBy(new Rotation2d(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD));
        var rotatedBySetpoint     = rotatedByTurretOffset.rotateBy(new Rotation2d(rawSetpoint));

        return robotPose.plus(new Transform2d(rotatedBySetpoint, new Rotation2d()));
    }

    private Translation2d getHubTranslationInRobotFrame(SwerveDriveState swerveState, Translation2d hubCoordinates)
    {
        var robotToHubField = hubCoordinates.minus(swerveState.Pose.getTranslation());

        return robotToHubField.rotateBy(swerveState.Pose.getRotation().unaryMinus());
    }

    private Angle getTurretSetpointForRobotFrameTarget(Translation2d targetTranslation)
    {
        return targetTranslation.getAngle().getMeasure().minus(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD);
    }

    private Angle getTurretSetpointForFieldTarget(Pose2d robotPose, Angle fieldTargetAngle)
    {
        return fieldTargetAngle.minus(robotPose.getRotation().getMeasure()).minus(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD);
    }

    private Angle getLegacyReleaseLateralCorrection(Angle rawSetpoint, Distance distance)
    {
        var safeDistanceM        = Math.max(distance.in(Meters), 0.5);
        var releaseLateralOffset = ShooterConstants.TURRET_PIVOT_TO_RELEASE.getY();
        var lateralErrorM        = releaseLateralOffset * Math.sin(Math.toRadians(rawSetpoint.in(Degrees)));

        return Degrees.of(Math.toDegrees(Math.atan2(lateralErrorM, safeDistanceM)));
    }

    private MovingTrackState calculateMovingTrackState(SwerveDriveState swerveState, Translation2d hubCoordinates)
    {
        // This follows the turret-origin solve pattern described publicly by 5000
        // and 6328: predict the release pose, compute turret-point velocity as
        // v + omega x r, then iterate TOF lookahead from the turret pivot.
        var releasePose         = predictReleasePose(swerveState.Pose, swerveState.Speeds);
        var turretOriginField   = getPointInField(releasePose, ShooterConstants.ROBOT_TO_TURRET_PIVOT);
        var turretPointVelocity = getTurretPointVelocityField(releasePose, swerveState.Speeds, ShooterConstants.ROBOT_TO_TURRET_PIVOT);
        var lookaheadOrigin     = turretOriginField;
        var lookaheadDistance   = Meters.of(hubCoordinates.getDistance(lookaheadOrigin));
        var timeOfFlight        = ShooterConstants.getShotTimeOfFlight(lookaheadDistance);

        for (int i = 0; i < ShooterConstants.SWM_LOOKAHEAD_ITERATIONS; i++)
        {
            lookaheadOrigin   = turretOriginField.plus(turretPointVelocity.times(timeOfFlight.in(Seconds)));
            lookaheadDistance = Meters.of(hubCoordinates.getDistance(lookaheadOrigin));
            timeOfFlight      = ShooterConstants.getShotTimeOfFlight(lookaheadDistance);
        }

        return new MovingTrackState(releasePose, lookaheadOrigin, lookaheadDistance, timeOfFlight);
    }

    private Pose2d predictReleasePose(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds)
    {
        var phaseDelaySeconds = ShooterConstants.SWM_RELEASE_PHASE_DELAY.in(Seconds);

        return robotPose.exp(new Twist2d(robotRelativeSpeeds.vxMetersPerSecond * phaseDelaySeconds, robotRelativeSpeeds.vyMetersPerSecond * phaseDelaySeconds, robotRelativeSpeeds.omegaRadiansPerSecond * phaseDelaySeconds));
    }

    private Translation2d getPointInField(Pose2d robotPose, Translation2d robotRelativePoint)
    {
        return robotPose.getTranslation().plus(robotRelativePoint.rotateBy(robotPose.getRotation()));
    }

    private Translation2d getTurretPointVelocityField(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, Translation2d robotRelativeOffset)
    {
        var fieldRelativeRobotVelocity = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());
        var fieldRelativeOffset        = robotRelativeOffset.rotateBy(robotPose.getRotation());
        var omegaCrossR                = new Translation2d(-robotRelativeSpeeds.omegaRadiansPerSecond * fieldRelativeOffset.getY(), robotRelativeSpeeds.omegaRadiansPerSecond * fieldRelativeOffset.getX());

        return fieldRelativeRobotVelocity.plus(omegaCrossR);
    }

    private ShotSolution createIdleSolution(Pose2d robotPose)
    {
        logShotKinematics(robotPose, getPointInField(robotPose, ShooterConstants.ROBOT_TO_TURRET_PIVOT), Meters.zero(), Seconds.zero());
        return new ShotSolution(false, ShooterConstants.TURRET_HOME_ANGLE, Meters.zero(), ShotMode.Idle, robotPose, false);
    }

    private void logShotKinematics(Pose2d releasePose, Translation2d lookaheadTurretOrigin, Distance distance, Time timeOfFlight)
    {
        _predictedReleasePose      = releasePose;
        _lookaheadTurretOriginPose = new Pose2d(lookaheadTurretOrigin, new Rotation2d());
        _targetDistance            = distance;
        _shotTimeOfFlight          = timeOfFlight;
    }

    private AprilTagFiducial selectBestTag(List<Integer> targetTagIds, AprilTagFiducial... fiducials)
    {
        AprilTagFiducial bestTag = null;

        for (AprilTagFiducial tag : fiducials)
        {
            var tagId = (int)Math.round(tag.fiducialID);

            if (!targetTagIds.contains(tagId))
            {
                continue;
            }

            if (bestTag == null || tag.ta > bestTag.ta || tag.ta == bestTag.ta && Math.abs(tag.tx) < Math.abs(bestTag.tx))
            {
                bestTag = tag;
            }
        }

        return bestTag;
    }

    private void updateFilter(List<Integer> filters, boolean force)
    {
        if (!force && (!DriverStation.isDisabled() || filters.equals(_targetTagFilter)))
        {
            return;
        }

        _limelight.getSettings().withAprilTagIdFilter(filters).save();
        _targetTagFilter = List.copyOf(filters);
    }
}
