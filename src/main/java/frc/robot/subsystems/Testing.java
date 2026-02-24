package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;

public class Testing extends SubsystemBase
{
    private final Intake   _intake;
    private final Feeder   _feeder;
    private final Flywheel _flywheel;
    private final Hood     _hood;

    // array of testhook
    // current testhook
    // fixrate
    private final int _shiftMod;

    public Command commandShift()
    {
        return startEnd(() -> _shiftMod = 1, () -> _shiftMod = 0;
    }

    public Testing() 
    {
        _intake = new Intake(); 
        // myarray.append( intake.gethook() )
        // myarray.append( _intake.gethookext() ) ;
        _feeder = new Feeder(); 
        _flywheel = new Flywheel(); 
        _hood = new Hood(); 

        // fixrate = 0.1
        _shiftMod = 0;
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
