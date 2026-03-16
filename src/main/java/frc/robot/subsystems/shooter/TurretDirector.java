package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Turret.TurretState;
import frc.robot.util.Utilities;
import limelight.networktables.target.AprilTagFiducial;

public class TurretDirector
{
    private TurretDirector()
    {
    }

    public static Translation2d calculate(TurretState turretState, Rotation2d localTurretAngle, SwerveDriveState swerveState, AprilTagFiducial... fiducials)
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
                    double avgTx = 0.0;
                    double avgTy = 0.0;

                    for (AprilTagFiducial tag : fiducials)
                    {
                        // subtract the values instead of adding them
                        // our limelight is mounted upside down so we
                        // need the inverse of these measurements
                        avgTx -= tag.tx;
                        avgTy -= tag.ty;
                    }

                    int n = fiducials.length;

                    avgTx /= n;
                    avgTy /= n;

                    // Tx and Ty are now the average pitch and yaw to the target.
                    // Now calculate distance from pitch
                    var tyMeasure = Degrees.of(avgTy);
                    var distance  = ShooterConstants.TURRET_TO_HUB_HEIGHT_DELTA.div(Math.tan(tyMeasure.plus(ShooterConstants.TURRET_LIMELIGHT_PITCH).in(Radians)));

                    // We have a distance and an angle. We can now describe the camera to target
                    // translation as a forward vector with magnitude "distance" rotated by angle
                    // "avgTx"

                    var localCameraToTarget = new Translation2d(distance, Inches.zero()).rotateBy(Rotation2d.fromDegrees(-avgTx));

                    // now, calculate ret and account for the turret being offset and at an angle
                    // get translation from the center of the robot to the camera
                    var cameraPosition = ShooterConstants.TURRET_POSITION.plus(ShooterConstants.TURRET_CAMERA_POSITION.rotateBy(localTurretAngle));
                    ret = cameraPosition.plus(localCameraToTarget.rotateBy(localTurretAngle).rotateBy(Rotation2d.fromDegrees(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD.in(Degrees))));
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
