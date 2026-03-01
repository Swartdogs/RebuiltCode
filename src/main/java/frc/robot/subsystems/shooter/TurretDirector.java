package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.GeneralConstants;
import frc.robot.subsystems.shooter.Turret.TurretState;
import frc.robot.util.Utilities;
import limelight.networktables.target.AprilTagFiducial;

public class TurretDirector
{
    private TurretDirector()
    {
    }

    public static Pose2d calculate(TurretState turretState, SwerveDriveState swerveState, AprilTagFiducial... fiducials)
    {
        Pose2d ret;

        switch (turretState)
        {
            case Track:
                // No tags, get angle from current pose to hub
                if (fiducials.length <= 0)
                {
                    var hub        = Utilities.getHubCoordinates();
                    var robot      = swerveState.Pose.getTranslation();
                    var robotToHub = hub.minus(robot);
                    var angle      = Rotation2d.fromRadians(Math.atan2(robotToHub.getY(), robotToHub.getX()));

                    ret = new Pose2d(robotToHub, angle);
                }
                else
                {
                    var sumX   = 0.0;
                    var sumY   = 0.0;
                    var sumCos = 0.0;
                    var sumSin = 0.0;

                    for (var tag : fiducials)
                    {
                        var pose = tag.getTargetPose_CameraSpace2D();

                        sumX   += pose.getX();
                        sumY   += pose.getY();
                        sumCos += pose.getRotation().getCos();
                        sumSin += pose.getRotation().getSin();
                    }

                    int n = fiducials.length;

                    ret = new Pose2d(sumX / n, sumY / n, new Rotation2d(sumCos, sumSin));
                }
                break;

            case Pass:
                Rotation2d rotation;
                Translation2d translation;

                if (Utilities.isBlueAlliance())
                {
                    // Only worry about X distance. Shoot perpendicular to driver station
                    translation = new Translation2d(Meters.of(-swerveState.Pose.getX()), Meters.zero());
                    rotation    = Rotation2d.kZero;
                }
                else
                {
                    // Only worry about X distance. Shoot perpendicular to driver station
                    translation = new Translation2d(GeneralConstants.FIELD_SIZE_X.minus(swerveState.Pose.getMeasureX()), Meters.zero());
                    rotation    = Rotation2d.k180deg;
                }

                ret = new Pose2d(translation, rotation);
                break;

            case Idle:
            default:
                ret = new Pose2d();
                break;
        }

        return ret;
    }
}
