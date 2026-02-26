package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.shooter.Turret.TurretState;
import limelight.networktables.target.AprilTagFiducial;

public class TurretDirector
{
    public record HubTagPair(AprilTagFiducial center, AprilTagFiducial offset)
    {
    }

    private TurretDirector()
    {
    }

    public static Angle calculate(TurretState turretState, SwerveDriveState swerveState, List<HubTagPair> hubTagPairs)
    {
        return switch (turretState)
        {
            case Idle -> null;
            default -> Degrees.zero();
        };
    }
}
