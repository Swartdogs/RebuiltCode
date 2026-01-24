package frc.robot.subsystems;

import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;

import frc.robot.Constants;

public class TurretTracking extends SubsystemBase
{
    private Limelight _trackingLimelight;
    @Logged
    private double    tx;
    @Logged
    private double    ty;
    private final PIDController _turnAnglePID; 

    public TurretTracking()
    {
        _trackingLimelight = new Limelight(Constants.Vision.TRACKING_CAMERA_NAME);
        _turnAnglePID      = new PIDController(Constants.Vision.TURN_ANGLE_KP, Constants.Vision.TURN_ANGLE_KI, Constants.Vision.TURN_ANGLE_KD);
    }

    @Override
    public void periodic()
    {
        tx = _trackingLimelight.getData().targetData.getHorizontalOffset();
        ty = _trackingLimelight.getData().targetData.getVerticalOffset();
    }

    public double getTX()
    {
        return _trackingLimelight.getData().targetData.getHorizontalOffset();
    }

    public double getTY()
    {
        return _trackingLimelight.getData().targetData.getVerticalOffset();
    }

    public double getNewRotationalRate() 
    {
        return _turnAnglePID.calculate(tx);
    }
}
