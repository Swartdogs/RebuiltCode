package frc.robot.subsystems;

import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;

import frc.robot.Constants;

public class TurretTracking extends SubsystemBase
{
    private Limelight _trackingLimelight;
    @Logged
    private double    _tx;

    public TurretTracking()
    {
        _trackingLimelight = new Limelight(Constants.Vision.RIGHT_CAMERA_NAME); // Temporary, will be changed
    }

    @Override
    public void periodic()
    {
        var pose = _trackingLimelight.getData().targetData.getCameraToTarget(); 
    }

    public double getTX()
    {
        return _trackingLimelight.getData().targetData.getHorizontalOffset();
    }

    public double getTY()
    {
        return _trackingLimelight.getData().targetData.getVerticalOffset();
    }
}
