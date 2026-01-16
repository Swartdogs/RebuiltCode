package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO
{
    @Logged
    public static class VisionIOInputs
    {
        public double  captureTimestamp = 0.0;
        public Pose2d  pose             = new Pose2d();
        public boolean hasPose          = false;
        public int     tagCount         = 0;
        public double  avgTagDistance   = 0.0;
    }

    public default void updateInputs(VisionIOInputs inputs)
    {
    }
}
