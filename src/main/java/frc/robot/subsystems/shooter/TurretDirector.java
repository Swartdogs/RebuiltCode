package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    public static Transform2d calculate(TurretState turretState, SwerveDriveState swerveState, AprilTagFiducial... fiducials)
    {
        Transform2d ret;

        switch (turretState)
        {
            case Track:
                // No tags, get angle from current pose to hub
                if (fiducials.length <= 0)
                {
                    Translation2d hub                  = Utilities.getHubCoordinates();
                    Translation2d robot                = swerveState.Pose.getTranslation();
                    Translation2d robotToHub           = hub.minus(robot);
                    Rotation2d    angle                = Rotation2d.fromRadians(Math.atan2(robotToHub.getY(), robotToHub.getX()));
                    Rotation2d    robotAngleCorrection = swerveState.Pose.getRotation().unaryMinus();

                    ret = new Transform2d(robotToHub.rotateBy(robotAngleCorrection), angle.rotateBy(robotAngleCorrection));
                }
                else
                {
                    double sumX   = 0.0;
                    double sumY   = 0.0;
                    double sumCos = 0.0;
                    double sumSin = 0.0;

                    for (AprilTagFiducial tag : fiducials)
                    {
                        Pose2d pose = tag.getTargetPose_CameraSpace2D();

                        sumX   += pose.getX();
                        sumY   += pose.getY();
                        sumCos += pose.getRotation().getCos();
                        sumSin += pose.getRotation().getSin();
                    }

                    int n = fiducials.length;

                    ret = new Transform2d(sumX / n, sumY / n, new Rotation2d(sumCos, sumSin));
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

                ret = new Transform2d(translation, rotation);
                break;

            case Idle:
            default:
                ret = new Transform2d();
                break;
        }

        return ret;
    }
}
