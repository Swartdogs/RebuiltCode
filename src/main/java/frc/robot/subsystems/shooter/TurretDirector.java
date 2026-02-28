package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Utilities;
import limelight.networktables.target.AprilTagFiducial;

public final class TurretDirector
{
    private static final List<Integer> kRedCenterTags  = List.of(3, 5, 9, 11);
    private static final List<Integer> kBlueCenterTags = List.of(20, 26, 21, 19); // TODO: confirm these tag IDs (red and blue)

    private TurretDirector()
    {
    }

    public static List<Integer> getAllianceCenterTags()
    {
        return Utilities.isRedAlliance() ? kRedCenterTags : kBlueCenterTags;
    }

    public static Double averageCenterTagPose(AprilTagFiducial[] fiducials)
    {
        if (fiducials == null || fiducials.length == 0)
        {
            return null;
        }

        var    centerTags = getAllianceCenterTags();
        double sum        = 0;
        int    count      = 0;

        for (var fiducial : fiducials)
        {
            if (!centerTags.contains((int)fiducial.fiducialID))
            {
                continue;
            }

            sum += fiducial.getTargetPose_CameraSpace2D().getTranslation().getNorm();
            count++;
        }

        return count > 0 ? sum / count : null;
    }

    public static Angle passTargetRobotFrame(SwerveDriveState swerveState)
    {
        return ShooterConstants.TURRET_PASS_TARGET.minus(swerveState.Pose.getRotation().getMeasure());
    }

    public static Angle calculateTrackSetpoint(Angle currentRobotAngle, double horizontalOffsetDegrees)
    {
        return currentRobotAngle.plus(Degrees.of(horizontalOffsetDegrees));
    }
}
