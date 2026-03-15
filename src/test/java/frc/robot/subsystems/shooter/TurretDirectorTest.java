package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Turret.TurretState;
import frc.robot.util.MeasureUtil;
import limelight.networktables.target.AprilTagFiducial;

class TurretDirectorTest
{
    private static final double  kAngleToleranceDeg    = 1e-9;
    private static final double  kDistanceToleranceM   = 1e-9;
    private static final double  kTranslationTolerance = 1e-9;
    private final TurretDirector _turretDirector       = new TurretDirector();

    @Test
    void trackStaticShotMatchesLegacyGeometryWithoutVision()
    {
        var state    = createState(new Pose2d(3.1, 4.2, Rotation2d.fromDegrees(32.0)));
        var actual   = _turretDirector.calculate(TurretState.Track, state, ShooterConstants.BLUE_HUB, ShooterConstants.BLUE_HUB_TAG_IDS, true, true);
        var expected = legacyTrackSolution(state, ShooterConstants.BLUE_HUB, true);

        assertSolutionMatches(expected, actual);
    }

    @Test
    void trackStaticShotPreservesHubValidityWithoutLookahead()
    {
        var state    = createState(new Pose2d(5.0, 2.0, Rotation2d.fromDegrees(-18.0)));
        var actual   = _turretDirector.calculate(TurretState.Track, state, ShooterConstants.RED_HUB, ShooterConstants.RED_HUB_TAG_IDS, false, false);
        var expected = legacyTrackSolution(state, ShooterConstants.RED_HUB, false);

        assertSolutionMatches(expected, actual);
    }

    @Test
    void passShotMatchesLegacyBlueAllianceBehavior()
    {
        var state    = createState(new Pose2d(2.4, 5.5, Rotation2d.fromDegrees(47.0)));
        var actual   = _turretDirector.calculate(TurretState.Pass, state, ShooterConstants.BLUE_HUB, ShooterConstants.BLUE_HUB_TAG_IDS, true, true);
        var expected = legacyPassSolution(state, true);

        assertSolutionMatches(expected, actual);
    }

    @Test
    void passShotMatchesLegacyRedAllianceBehavior()
    {
        var state    = createState(new Pose2d(10.2, 3.3, Rotation2d.fromDegrees(-21.0)));
        var actual   = _turretDirector.calculate(TurretState.Pass, state, ShooterConstants.RED_HUB, ShooterConstants.RED_HUB_TAG_IDS, false, true);
        var expected = legacyPassSolution(state, false);

        assertSolutionMatches(expected, actual);
    }

    @Test
    void trackShotUsesBestAllowedVisionTagForTrim()
    {
        var state    = createState(new Pose2d(3.6, 4.8, Rotation2d.fromDegrees(14.0)));
        var actual   = _turretDirector.calculate(TurretState.Track, state, ShooterConstants.BLUE_HUB, ShooterConstants.BLUE_HUB_TAG_IDS, true, true, createTag(999, 50.0, -20.0), createTag(20, 4.0, 6.0), createTag(18, 8.0, 3.0));
        var expected = legacyTrackSolution(state, ShooterConstants.BLUE_HUB, true, 3.0, true);

        assertSolutionMatches(expected, actual);
    }

    private ShotSolution legacyTrackSolution(SwerveDriveState swerveState, Translation2d hubCoordinates, boolean hubActive)
    {
        return legacyTrackSolution(swerveState, hubCoordinates, hubActive, 0.0, false);
    }

    private ShotSolution legacyTrackSolution(SwerveDriveState swerveState, Translation2d hubCoordinates, boolean hubActive, double txDegrees, boolean hasVisionTarget)
    {
        var robotToHubField        = hubCoordinates.minus(swerveState.Pose.getTranslation());
        var localHubTranslation    = robotToHubField.rotateBy(swerveState.Pose.getRotation().unaryMinus());
        var turretToHubTranslation = localHubTranslation.minus(ShooterConstants.TURRET_POSITION);
        var distance               = Meters.of(turretToHubTranslation.getNorm());
        var rawSetpoint            = turretToHubTranslation.getAngle().getMeasure().minus(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD);

        if (hasVisionTarget && !MathUtil.isNear(0.0, txDegrees, ShooterConstants.TURRET_TRACK_TX_DEADBAND.in(Degrees)))
        {
            rawSetpoint = rawSetpoint.plus(Degrees.of(txDegrees));
        }

        var safeDistanceM = Math.max(distance.in(Meters), 0.5);
        var lateralErrorM = ShooterConstants.SHOOTER_LATERAL_OFFSET.in(Meters) * Math.sin(Math.toRadians(rawSetpoint.in(Degrees)));
        rawSetpoint = rawSetpoint.plus(Degrees.of(Math.toDegrees(Math.atan2(lateralErrorM, safeDistanceM))));

        return new ShotSolution(hubActive, clampTurretAngle(rawSetpoint), distance, TurretState.Track, buildTargetPose(swerveState.Pose, distance, rawSetpoint), hasVisionTarget);
    }

    private ShotSolution legacyPassSolution(SwerveDriveState swerveState, boolean isBlueAlliance)
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

        return new ShotSolution(true, clampTurretAngle(rawSetpoint), distance, TurretState.Pass, buildTargetPose(swerveState.Pose, distance, rawSetpoint), false);
    }

    private Pose2d buildTargetPose(Pose2d robotPose, Distance distance, Angle rawSetpoint)
    {
        var forwardTranslation    = new Translation2d(distance, Meters.zero());
        var rotatedByTurretOffset = forwardTranslation.rotateBy(new Rotation2d(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD));
        var rotatedBySetpoint     = rotatedByTurretOffset.rotateBy(new Rotation2d(rawSetpoint));

        return robotPose.plus(new Transform2d(rotatedBySetpoint, new Rotation2d()));
    }

    private Angle clampTurretAngle(Angle rawSetpoint)
    {
        var wrappedDegrees = MathUtil.inputModulus(rawSetpoint.in(Degrees), -180, 180);
        return MeasureUtil.clamp(Degrees.of(wrappedDegrees), ShooterConstants.TURRET_SOFT_MIN_ANGLE, ShooterConstants.TURRET_SOFT_MAX_ANGLE);
    }

    private SwerveDriveState createState(Pose2d pose)
    {
        var state = new SwerveDriveState();
        state.Pose   = pose;
        state.Speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        return state;
    }

    private AprilTagFiducial createTag(int fiducialId, double ta, double tx)
    {
        var tag = new AprilTagFiducial();
        tag.fiducialID = fiducialId;
        tag.ta         = ta;
        tag.tx         = tx;
        return tag;
    }

    private void assertSolutionMatches(ShotSolution expected, ShotSolution actual)
    {
        assertEquals(expected.valid(), actual.valid());
        assertEquals(expected.hasVisionTarget(), actual.hasVisionTarget());
        assertEquals(expected.mode(), actual.mode());
        assertEquals(expected.turretAngle().in(Degrees), actual.turretAngle().in(Degrees), kAngleToleranceDeg);
        assertEquals(expected.distance().in(Meters), actual.distance().in(Meters), kDistanceToleranceM);
        assertEquals(expected.targetPose().getX(), actual.targetPose().getX(), kTranslationTolerance);
        assertEquals(expected.targetPose().getY(), actual.targetPose().getY(), kTranslationTolerance);
        assertEquals(expected.targetPose().getRotation().getDegrees(), actual.targetPose().getRotation().getDegrees(), kAngleToleranceDeg);
    }
}
