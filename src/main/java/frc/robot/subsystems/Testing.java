package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;

public class Testing extends SubsystemBase 
{
    private final Intake _intake; 
    private final Feeder _feeder; 
    private final Flywheel _flywheel; 
    private final Hood _hood; 

    public Testing() 
    {
        _intake = new Intake(); 
        _feeder = new Feeder(); 
        _flywheel = new Flywheel(); 
        _hood = new Hood(); 
    }

    @Override
    public void periodic()
    {

    }

    public void stopAll() 
    {
        _intake.stopRollers(); 
    }
}
