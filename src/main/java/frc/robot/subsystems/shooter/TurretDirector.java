package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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

    public record ShotSolution(boolean valid, Angle turretAngle, Distance distance, ShotMode mode, Pose2d targetPose, boolean hasVisionTarget, boolean inRange, boolean hubActive)
    {
    }

    private record MovingTrackState(Pose2d releasePose, Translation2d lookaheadTurretOrigin, Distance lookaheadDistance, Time timeOfFlight)
    {
    }

    private final Supplier<SwerveDriveState> _swerveStateSupplier;
    private final Limelight                  _limelight;
    private final LinearFilter               _visionTxFilter;
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
    private Angle                            _desiredTurretAngle;
    @Logged
    private Pose2d                           _predictedReleasePose;
    @Logged
    private Pose2d                           _lookaheadTurretOriginPose;
    @Logged
    private Time                             _shotTimeOfFlight;
    @Logged
    private Angle                            _visionTxRaw;
    @Logged
    private Angle                            _visionTxFiltered;
    @Logged
    private Angle                            _visionTxApplied;
    @Logged
    private int                              _selectedVisionTagId;
    @Logged
    private Angle                            _poseOnlyTrackAngle;
    @Logged
    private Angle                            _poseVisionTrackDelta;
    @Logged
    private Angle                            _empiricalDriftCorrection;
    @Logged
    private double                           _empiricalDriftLateralErrorInches;
    @Logged
    private double                           _empiricalDriftSign;
    @Logged
    private Angle                            _trackCommandDelta;

    public TurretDirector(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        var txFilterWindowSamples = Math.max(1, (int)Math.round(ShooterConstants.TURRET_VISION_TX_FILTER_WINDOW.in(Seconds) / GeneralConstants.LOOP_PERIOD.in(Seconds)));
        _swerveStateSupplier              = swerveStateSupplier;
        _limelight                        = new Limelight(ShooterConstants.LIMELIGHT_NAME);
        _visionTxFilter                   = LinearFilter.movingAverage(txFilterWindowSamples);
        _targetTagFilter                  = List.of();
        _currentSolution                  = new ShotSolution(false, ShooterConstants.TURRET_HOME_ANGLE, Meters.zero(), ShotMode.Idle, new Pose2d(), false, false, false);
        _targetDistance                   = Meters.zero();
        _targetPose                       = new Pose2d();
        _limelightHasTarget               = false;
        _hubActive                        = false;
        _targetValid                      = false;
        _desiredTurretAngle               = ShooterConstants.TURRET_HOME_ANGLE;
        _predictedReleasePose             = new Pose2d();
        _lookaheadTurretOriginPose        = new Pose2d();
        _shotTimeOfFlight                 = Seconds.zero();
        _visionTxRaw                      = Degrees.zero();
        _visionTxFiltered                 = Degrees.zero();
        _visionTxApplied                  = Degrees.zero();
        _selectedVisionTagId              = -1;
        _poseOnlyTrackAngle               = ShooterConstants.TURRET_HOME_ANGLE;
        _poseVisionTrackDelta             = Degrees.zero();
        _empiricalDriftCorrection         = Degrees.zero();
        _empiricalDriftLateralErrorInches = 0.0;
        _empiricalDriftSign               = ShooterConstants.TURRET_LINEAR_DRIFT_CORRECTION_SIGN;
        _trackCommandDelta                = Degrees.zero();

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
        solution            = stabilizeTrackCommand(solution);
        _currentSolution    = solution;
        _targetDistance     = solution.distance();
        _targetPose         = solution.targetPose();
        _limelightHasTarget = solution.hasVisionTarget();
        _targetValid        = solution.valid();
        _desiredTurretAngle = solution.turretAngle();
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

    private ShotSolution stabilizeTrackCommand(ShotSolution solution)
    {
        if (solution.mode() != ShotMode.Track || _currentSolution.mode() != ShotMode.Track || !_currentSolution.valid())
        {
            _trackCommandDelta = Degrees.zero();
            return solution;
        }

        var requestedDeg = solution.turretAngle().in(Degrees);
        var previousDeg  = _currentSolution.turretAngle().in(Degrees);
        var deltaDeg     = MathUtil.inputModulus(requestedDeg - previousDeg, -180.0, 180.0);
        _trackCommandDelta = Degrees.of(deltaDeg);

        var deadbandDeg = ShooterConstants.TURRET_TRACK_COMMAND_DEADBAND.in(Degrees);

        if (Math.abs(deltaDeg) <= deadbandDeg)
        {
            return new ShotSolution(solution.valid(), _currentSolution.turretAngle(), solution.distance(), solution.mode(), solution.targetPose(), solution.hasVisionTarget(), solution.inRange(), solution.hubActive());
        }

        var maxStepDeg = ShooterConstants.TURRET_TRACK_COMMAND_MAX_STEP_PER_LOOP.in(Degrees);
        var stepDeg    = MathUtil.clamp(deltaDeg, -maxStepDeg, maxStepDeg);
        var stableDeg  = previousDeg + stepDeg;

        return new ShotSolution(solution.valid(), Degrees.of(stableDeg), solution.distance(), solution.mode(), solution.targetPose(), solution.hasVisionTarget(), solution.inRange(), solution.hubActive());
    }

    private ShotSolution calculateTrack(SwerveDriveState swerveState, Translation2d hubCoordinates, List<Integer> targetTagIds, boolean hubActive, AprilTagFiducial... fiducials)
    {
        var turretReleaseField = getPointInField(swerveState.Pose, ShooterConstants.ROBOT_TO_TURRET_RELEASE);
        var releaseToHubField  = hubCoordinates.minus(turretReleaseField);
        var distance           = Meters.of(releaseToHubField.getNorm());
        var rawSetpoint        = getTurretSetpointForFieldTarget(swerveState.Pose, releaseToHubField.getAngle().getMeasure());
        _poseOnlyTrackAngle = rawSetpoint;
        var bestTag = selectBestTag(targetTagIds, fiducials);
        rawSetpoint           = applyVisionTrim(rawSetpoint, bestTag);
        rawSetpoint           = rawSetpoint.plus(ShooterConstants.TURRET_TRACK_BIAS);
        rawSetpoint           = rawSetpoint.plus(getEmpiricalLinearDriftCorrection(rawSetpoint, distance));
        _poseVisionTrackDelta = rawSetpoint.minus(_poseOnlyTrackAngle);

        logShotKinematics(swerveState.Pose, turretReleaseField, distance, ShooterConstants.getShotTimeOfFlight(distance));

        return new ShotSolution(true, rawSetpoint, distance, ShotMode.Track, buildTargetPose(swerveState.Pose, distance, rawSetpoint), bestTag != null, true, hubActive);
    }

    private Angle applyVisionTrim(Angle rawSetpoint, AprilTagFiducial bestTag)
    {
        if (bestTag == null)
        {
            _visionTxRaw      = Degrees.zero();
            _visionTxFiltered = Degrees.zero();
            _visionTxApplied  = Degrees.zero();
            _visionTxFilter.reset();
            return rawSetpoint;
        }

        var rawTxDegrees = bestTag.tx;
        _visionTxRaw      = Degrees.of(rawTxDegrees);
        _visionTxFiltered = Degrees.of(_visionTxFilter.calculate(rawTxDegrees));

        if (!ShooterConstants.TURRET_USE_VISION_TX_TRIM)
        {
            _visionTxApplied = Degrees.zero();
            return rawSetpoint;
        }

        var desiredTxDegrees = _visionTxFiltered.in(Degrees);

        if (MathUtil.isNear(0.0, desiredTxDegrees, ShooterConstants.TURRET_TRACK_TX_DEADBAND.in(Degrees)))
        {
            desiredTxDegrees = 0.0;
        }

        var previousTxDegrees = _visionTxApplied.in(Degrees);
        var deltaTxDegrees    = desiredTxDegrees - previousTxDegrees;
        var maxStepDegrees    = ShooterConstants.TURRET_VISION_TX_MAX_STEP_PER_LOOP.in(Degrees);
        var stepTxDegrees     = MathUtil.clamp(deltaTxDegrees, -maxStepDegrees, maxStepDegrees);
        _visionTxApplied = Degrees.of(previousTxDegrees + stepTxDegrees);

        return rawSetpoint.plus(_visionTxApplied);
    }

    private Angle getEmpiricalLinearDriftCorrection(Angle commandedAngle, Distance distance)
    {
        if (!ShooterConstants.TURRET_USE_LINEAR_DRIFT_COMPENSATION)
        {
            _empiricalDriftCorrection         = Degrees.zero();
            _empiricalDriftLateralErrorInches = 0.0;
            return Degrees.zero();
        }

        _empiricalDriftSign = ShooterConstants.TURRET_LINEAR_DRIFT_CORRECTION_SIGN;

        var commandedDegrees          = commandedAngle.in(Degrees);
        var rawLateralOffsetInches    = ShooterConstants.TURRET_DRIFT_LATERAL_BIAS.in(edu.wpi.first.units.Units.Inches)
                + ShooterConstants.TURRET_DRIFT_LATERAL_SINE_AMPLITUDE.in(edu.wpi.first.units.Units.Inches) * Math.sin(Math.toRadians(commandedDegrees + ShooterConstants.TURRET_DRIFT_LATERAL_PHASE_OFFSET.in(Degrees)));
        var signedLateralOffsetInches = _empiricalDriftSign * rawLateralOffsetInches;
        var distanceMeters            = Math.max(distance.in(Meters), 0.25);
        var signedCorrectionDegrees   = Math.toDegrees(Math.atan2(Units.inchesToMeters(signedLateralOffsetInches), distanceMeters));
        var maxCorrectionDeg          = ShooterConstants.TURRET_LINEAR_DRIFT_MAX_CORRECTION.in(Degrees);
        signedCorrectionDegrees = MathUtil.clamp(signedCorrectionDegrees, -maxCorrectionDeg, maxCorrectionDeg);

        _empiricalDriftLateralErrorInches = signedLateralOffsetInches;
        _empiricalDriftCorrection         = Degrees.of(signedCorrectionDegrees);
        return _empiricalDriftCorrection;
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

        return new ShotSolution(true, rawSetpoint, distance, ShotMode.Pass, buildTargetPose(swerveState.Pose, distance, rawSetpoint), false, true, true);
    }

    private ShotSolution calculateMovingTrack(SwerveDriveState swerveState, Translation2d hubCoordinates, List<Integer> targetTagIds, boolean hubActive, AprilTagFiducial... fiducials)
    {
        var movingTrackState = calculateMovingTrackState(swerveState, hubCoordinates);
        var lookaheadToHub   = hubCoordinates.minus(movingTrackState.lookaheadTurretOrigin());
        var rawSetpoint      = getTurretSetpointForFieldTarget(movingTrackState.releasePose(), lookaheadToHub.getAngle().getMeasure());
        var bestTag          = selectBestTag(targetTagIds, fiducials);
        rawSetpoint                       = rawSetpoint.plus(ShooterConstants.TURRET_TRACK_BIAS);
        _poseVisionTrackDelta             = Degrees.zero();
        _empiricalDriftCorrection         = Degrees.zero();
        _empiricalDriftLateralErrorInches = 0.0;

        logShotKinematics(movingTrackState.releasePose(), movingTrackState.lookaheadTurretOrigin(), movingTrackState.lookaheadDistance(), movingTrackState.timeOfFlight());

        var inRange = ShooterConstants.isMovingShotDistanceValid(movingTrackState.lookaheadDistance());

        return new ShotSolution(
                inRange, rawSetpoint, movingTrackState.lookaheadDistance(), ShotMode.Track, buildTargetPose(movingTrackState.releasePose(), movingTrackState.lookaheadDistance(), rawSetpoint), bestTag != null, inRange, hubActive
        );
    }

    private Pose2d buildTargetPose(Pose2d robotPose, Distance distance, Angle rawSetpoint)
    {
        var forwardTranslation    = new Translation2d(distance, Meters.zero());
        var rotatedByTurretOffset = forwardTranslation.rotateBy(new Rotation2d(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD));
        var rotatedBySetpoint     = rotatedByTurretOffset.rotateBy(new Rotation2d(rawSetpoint));

        return robotPose.plus(new Transform2d(rotatedBySetpoint, new Rotation2d()));
    }

    private Angle getTurretSetpointForFieldTarget(Pose2d robotPose, Angle fieldTargetAngle)
    {
        return fieldTargetAngle.minus(robotPose.getRotation().getMeasure()).minus(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD);
    }

    private MovingTrackState calculateMovingTrackState(SwerveDriveState swerveState, Translation2d hubCoordinates)
    {
        var releasePose         = predictReleasePose(swerveState.Pose, swerveState.Speeds);
        var turretOriginField   = getPointInField(releasePose, ShooterConstants.ROBOT_TO_TURRET_RELEASE);
        var turretPointVelocity = getTurretPointVelocityField(releasePose, swerveState.Speeds, ShooterConstants.ROBOT_TO_TURRET_RELEASE);
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
        return new ShotSolution(false, ShooterConstants.TURRET_HOME_ANGLE, Meters.zero(), ShotMode.Idle, robotPose, false, false, false);
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
        AprilTagFiducial bestTag   = null;
        AprilTagFiducial lockedTag = null;

        for (AprilTagFiducial tag : fiducials)
        {
            var tagId = (int)Math.round(tag.fiducialID);

            if (!targetTagIds.contains(tagId))
            {
                continue;
            }

            if (tagId == _selectedVisionTagId)
            {
                lockedTag = tag;
            }

            if (bestTag == null || tag.ta > bestTag.ta || tag.ta == bestTag.ta && Math.abs(tag.tx) < Math.abs(bestTag.tx))
            {
                bestTag = tag;
            }
        }

        var selectedTag = lockedTag != null ? lockedTag : bestTag;

        if (selectedTag == null)
        {
            _selectedVisionTagId = -1;
            return null;
        }

        _selectedVisionTagId = (int)Math.round(selectedTag.fiducialID);
        return selectedTag;
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
