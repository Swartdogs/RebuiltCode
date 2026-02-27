package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Turret.TurretState;

public final class TurretDirector
{
    private TurretDirector()
    {
    }

    public static List<List<Integer>> getHubCenterOffsetPairs(boolean isRedAlliance)
    {
        if (isRedAlliance)
        {
            return ShooterConstants.RED_HUB_CENTER_OFFSET_TAG_PAIRS;
        }
        return ShooterConstants.BLUE_HUB_CENTER_OFFSET_TAG_PAIRS;
    }

    public static TurretAimSolution getAimSolution(DirectorContext context)
    {
        return switch (context.turretState())
        {
            case Idle -> TurretAimSolution.none();
            case Track -> getTrackSolution(context);
            case Pass -> TurretAimSolution.of(ShooterConstants.TURRET_PASS_TARGET, 0.0);
        };
    }

    private static TurretAimSolution getTrackSolution(DirectorContext context)
    {
        if (context.centerTagObservation() != null && context.offsetTagObservation() != null)
        {
            TriangulationResult triangulation = triangulate(context.centerTagObservation(), context.offsetTagObservation());
            if (triangulation.valid())
            {
                return TurretAimSolution.of(context.currentFieldAngle().plus(Degrees.of(triangulation.relativeAngleDeg())), triangulation.distanceMeters());
            }
        }

        if (context.hasTarget())
        {
            double fallbackDistance = context.centerTagObservation() == null ? 0.0 : context.centerTagObservation().distanceMeters();
            return TurretAimSolution.of(context.currentFieldAngle().plus(context.horizontalOffset()), fallbackDistance);
        }

        return TurretAimSolution.of(context.robotHeading().plus(Degrees.of(ShooterConstants.TURRET_HOME_ANGLE)), 0.0);
    }

    /**
     * Solves for robot-to-hub angle and distance using two known triangles.
     * <p>
     * Triangle R-C-O: Robot (R), center tag (C), offset tag (O). We know RC and RO
     * from vision and CL from field drawings (TURRET_CL_METERS). Law of sines gives
     * angle at C.
     * <p>
     * Triangle R-C-H: Hub center (H) is TURRET_CH_METERS from C (field drawing).
     * Law of cosines gives RH (distance to hub). Law of sines gives angle at R from
     * R→C to R→H. We add the camera-to-center-tag angle (tx) to get full
     * robot-frame angle to hub.
     */
    private static TriangulationResult triangulate(TagObservation centerTagObservation, TagObservation offsetTagObservation)
    {
        double rcDistanceMeters = centerTagObservation.distanceMeters();
        double roDistanceMeters = offsetTagObservation.distanceMeters();
        if (rcDistanceMeters <= 0.0 || roDistanceMeters <= 0.0)
        {
            return TriangulationResult.invalid();
        }

        double angleLrcRad = Math.toRadians(offsetTagObservation.horizontalOffset().in(Degrees) - centerTagObservation.horizontalOffset().in(Degrees));
        double sinLcr      = (roDistanceMeters * Math.sin(angleLrcRad)) / ShooterConstants.TURRET_CL_METERS;
        double angleLcrRad = Math.asin(MathUtil.clamp(sinLcr, -1.0, 1.0));

        double angleLcrPlusNinetyRad = angleLcrRad + Math.PI / 2.0;
        double rhSquared             = (rcDistanceMeters * rcDistanceMeters) + (ShooterConstants.TURRET_CH_METERS * ShooterConstants.TURRET_CH_METERS)
                - (2.0 * rcDistanceMeters * ShooterConstants.TURRET_CH_METERS * Math.cos(angleLcrPlusNinetyRad));
        if (rhSquared <= 0.0 || !Double.isFinite(rhSquared))
        {
            return TriangulationResult.invalid();
        }

        double rhDistanceMeters = Math.sqrt(rhSquared);
        double sinHrc           = (ShooterConstants.TURRET_CH_METERS * Math.sin(angleLcrPlusNinetyRad)) / rhDistanceMeters;
        double angleHrcDeg      = Math.toDegrees(Math.asin(MathUtil.clamp(sinHrc, -1.0, 1.0)));

        double angleRhDeg = angleHrcDeg + centerTagObservation.horizontalOffset().in(Degrees);
        if (!Double.isFinite(angleRhDeg))
        {
            return TriangulationResult.invalid();
        }

        return TriangulationResult.valid(angleRhDeg, rhDistanceMeters);
    }

    public record DirectorContext(TurretState turretState, Angle currentFieldAngle, Angle robotHeading, Angle horizontalOffset, boolean hasTarget, TagObservation centerTagObservation, TagObservation offsetTagObservation)
    {
    }

    public record TurretAimSolution(boolean hasSetpoint, Angle fieldSetpoint, double distanceToHubMeters)
    {
        public static TurretAimSolution none()
        {
            return new TurretAimSolution(false, Degrees.zero(), 0.0);
        }

        public static TurretAimSolution of(Angle fieldSetpoint, double distanceToHubMeters)
        {
            return new TurretAimSolution(true, fieldSetpoint, distanceToHubMeters);
        }
    }

    public record TagObservation(int id, Angle horizontalOffset, double distanceMeters)
    {
    }

    private record TriangulationResult(boolean valid, double relativeAngleDeg, double distanceMeters)
    {
        private static TriangulationResult invalid()
        {
            return new TriangulationResult(false, 0.0, 0.0);
        }

        private static TriangulationResult valid(double relativeAngleDeg, double distanceMeters)
        {
            return new TriangulationResult(true, relativeAngleDeg, distanceMeters);
        }
    }
}
