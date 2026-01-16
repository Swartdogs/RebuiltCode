package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import limelight.Limelight;

public class Vision extends SubsystemBase {

    private Limelight _limelight = new Limelight("limelight");
    private CommandSwerveDrivetrain _drivetrain; 

    public Vision() {
        // TODO 
    }

    @Override
    public void periodic() {
        _drivetrain.addVisionMeasurement(null, 0.0); // TODO 
        // limelight has pose estimate with MegaTag2 according to their documentation 
    }

    public boolean hasTarget() {
        return false; // TODO 
    }

    public double getTxDegrees() {
        return 0.0; // TODO
    }
}
