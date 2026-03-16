package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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

    public TurretDirector(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        _swerveStateSupplier = swerveStateSupplier;
        _limelight           = new Limelight(ShooterConstants.LIMELIGHT_NAME);
        _targetTagFilter     = List.of();
        _currentSolution     = new ShotSolution(false, ShooterConstants.TURRET_HOME_ANGLE, Meters.zero(), ShotMode.Idle, new Pose2d(), false);
        _targetDistance      = Meters.zero();
        _targetPose          = new Pose2d();
        _limelightHasTarget  = false;
        _hubActive           = false;
        _targetValid         = false;

        _limelight.getSettings().withAprilTagOffset(ShooterConstants.CENTER_TAG_TO_HUB_CENTER_OFFSET).save();
        updateFilter(Utilities.getOurHubTagIds(), true);
    }

    public ShotSolution calculate(ShotMode shotMode)
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

        var solution = calculate(shotMode, swerveState, Utilities.getHubCoordinates(), targetTagIds, isBlueAlliance, _hubActive, fiducials);
        _currentSolution    = solution;
        _targetDistance     = solution.distance();
        _targetPose         = solution.targetPose();
        _limelightHasTarget = solution.hasVisionTarget();
        _targetValid        = solution.valid();
        return solution;
    }

    ShotSolution calculate(ShotMode shotMode, SwerveDriveState swerveState, Translation2d hubCoordinates, List<Integer> targetTagIds, boolean isBlueAlliance, boolean hubActive, AprilTagFiducial... fiducials)
    {
        return switch (shotMode)
        {
            case Track -> calculateTrack(swerveState, hubCoordinates, targetTagIds, hubActive, fiducials);
            case Pass -> calculatePass(swerveState, isBlueAlliance);
            case Idle -> new ShotSolution(false, ShooterConstants.TURRET_HOME_ANGLE, Meters.zero(), ShotMode.Idle, swerveState.Pose, false);
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

        return new ShotSolution(true, rawSetpoint, distance, ShotMode.Pass, buildTargetPose(swerveState.Pose, distance, rawSetpoint), false);
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

    private Angle getLegacyReleaseLateralCorrection(Angle rawSetpoint, Distance distance)
    {
        var safeDistanceM        = Math.max(distance.in(Meters), 0.5);
        var releaseLateralOffset = ShooterConstants.TURRET_PIVOT_TO_RELEASE.getY();
        var lateralErrorM        = releaseLateralOffset * Math.sin(Math.toRadians(rawSetpoint.in(Degrees)));

        return Degrees.of(Math.toDegrees(Math.atan2(lateralErrorM, safeDistanceM)));
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
