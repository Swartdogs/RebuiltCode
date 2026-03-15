package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Turret.TurretState;
import frc.robot.util.MeasureUtil;
import limelight.networktables.target.AprilTagFiducial;

public class TurretDirector
{
    public ShotSolution calculate(TurretState turretState, SwerveDriveState swerveState, Translation2d hubCoordinates, List<Integer> targetTagIds, boolean isBlueAlliance, boolean hubActive, AprilTagFiducial... fiducials)
    {
        switch (turretState)
        {
            case Track:
                return calculateTrack(swerveState, hubCoordinates, targetTagIds, hubActive, fiducials);

            case Pass:
                return calculatePass(swerveState, isBlueAlliance);

            case Idle:
            default:
                return new ShotSolution(false, ShooterConstants.TURRET_HOME_ANGLE, Meters.zero(), turretState, swerveState.Pose, false);
        }
    }

    private ShotSolution calculateTrack(SwerveDriveState swerveState, Translation2d hubCoordinates, List<Integer> targetTagIds, boolean hubActive, AprilTagFiducial... fiducials)
    {
        var localHubTranslation    = getHubTranslationInRobotFrame(swerveState, hubCoordinates);
        var turretToHubTranslation = localHubTranslation.minus(ShooterConstants.TURRET_POSITION);
        var distance               = Meters.of(turretToHubTranslation.getNorm());
        var rawSetpoint            = getTurretSetpointForRobotFrameTarget(turretToHubTranslation);
        var bestTag                = selectBestTag(targetTagIds, fiducials);

        if (bestTag != null)
        {
            var txCorrection = Degrees.of(-bestTag.tx);

            if (!MathUtil.isNear(0.0, txCorrection.in(Degrees), ShooterConstants.TURRET_TRACK_TX_DEADBAND.in(Degrees)))
            {
                rawSetpoint = rawSetpoint.minus(txCorrection);
            }
        }

        // Preserve the current static shooter-offset correction so the calculator
        // reproduces today's behavior before SWM math is introduced.
        var safeDistanceM = Math.max(distance.in(Meters), 0.5);
        var lateralErrorM = ShooterConstants.SHOOTER_LATERAL_OFFSET.in(Meters) * Math.sin(Math.toRadians(rawSetpoint.in(Degrees)));
        rawSetpoint = rawSetpoint.plus(Degrees.of(Math.toDegrees(Math.atan2(lateralErrorM, safeDistanceM))));

        return new ShotSolution(hubActive, clampTurretAngle(rawSetpoint), distance, TurretState.Track, buildTargetPose(swerveState.Pose, distance, rawSetpoint), bestTag != null);
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

        return new ShotSolution(true, clampTurretAngle(rawSetpoint), distance, TurretState.Pass, buildTargetPose(swerveState.Pose, distance, rawSetpoint), false);
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

    private Angle clampTurretAngle(Angle rawSetpoint)
    {
        var wrappedDegrees = MathUtil.inputModulus(rawSetpoint.in(Degrees), -180, 180);
        return MeasureUtil.clamp(Degrees.of(wrappedDegrees), ShooterConstants.TURRET_SOFT_MIN_ANGLE, ShooterConstants.TURRET_SOFT_MAX_ANGLE);
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
}
