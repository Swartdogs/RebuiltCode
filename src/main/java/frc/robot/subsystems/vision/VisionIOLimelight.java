package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO
{
    private final String              _cameraName;
    private final Supplier<Rotation2d> _robotYawSupplier;

    public VisionIOLimelight(String cameraName, Supplier<Rotation2d> robotYawSupplier)
    {
        _cameraName       = cameraName;
        _robotYawSupplier = robotYawSupplier;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs)
    {
        // Set robot orientation for MegaTag2, required before getting pose estimate
        LimelightHelpers.SetRobotOrientation(
            _cameraName,
            _robotYawSupplier.get().getDegrees(),
            0, 0, 0, 0, 0
        );

        var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(_cameraName);

        if (poseEstimate != null
            && poseEstimate.tagCount > 0
            && poseEstimate.avgTagDist < Constants.Vision.MAX_DETECTION_RANGE)
        {
            inputs.captureTimestamp = poseEstimate.timestampSeconds;
            inputs.pose             = poseEstimate.pose;
            inputs.hasPose          = true;
            inputs.tagCount         = poseEstimate.tagCount;
            inputs.avgTagDistance   = poseEstimate.avgTagDist;
        }
        else
        {
            inputs.hasPose = false;
        }
    }
}
