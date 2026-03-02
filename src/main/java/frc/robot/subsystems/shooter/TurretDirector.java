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

    public static Translation2d calculate(TurretState turretState, SwerveDriveState swerveState, AprilTagFiducial... fiducials)
    {
        Translation2d ret;

        switch (turretState)
        {
            case Track:
                // No tags, get angle from current pose to hub
                if (fiducials.length <= 0)
                {
                    Translation2d hub                  = Utilities.getHubCoordinates();
                    Translation2d robot                = swerveState.Pose.getTranslation();
                    Translation2d robotToHub           = hub.minus(robot);
                    Rotation2d    robotAngleCorrection = swerveState.Pose.getRotation().unaryMinus();

                    ret = robotToHub.rotateBy(robotAngleCorrection);
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

                    ret = new Translation2d(sumX / n, sumY / n).rotateBy(new Rotation2d(sumCos, sumSin));
                }
                break;

            case Pass:
                Rotation2d robotAngle = swerveState.Pose.getRotation();
                Rotation2d rotation;
                Translation2d translation;

                // The distance we need to aim for is determined by the robot's pose and the
                // alliance we're on.
                // The direction we need to aim is based on the alliance.
                // Describe the translation as a forward vector rotated by the robot's rotation
                if (Utilities.isBlueAlliance())
                {
                    translation = new Translation2d(Meters.of(swerveState.Pose.getX()), Meters.zero());
                    rotation    = Rotation2d.k180deg.minus(robotAngle);
                }
                else
                {
                    translation = new Translation2d(GeneralConstants.FIELD_SIZE_X.minus(swerveState.Pose.getMeasureX()), Meters.zero());
                    rotation    = robotAngle.unaryMinus();
                }

                ret = translation.rotateBy(rotation);
                break;

            case Idle:
            default:
                ret = new Translation2d();
                break;
        }

        return ret;
    }
}
