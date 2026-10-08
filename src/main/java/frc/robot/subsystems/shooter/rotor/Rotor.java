package frc.robot.subsystems.shooter.rotor;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.rotor.RotorIO.RotorIOInputs;

/*
 * Class containing the business logic for how the rotor component of the
 * shooter subsystem works
 */
@Logged
public class Rotor extends SubsystemBase
{
    // The inputs measured from the rotor motor
    @Logged
    private final RotorIOInputs _inputs;

    // The physical layer (motors and sensors) the business logic interacts with
    @NotLogged
    private final RotorIO _io;

    // Provides a clean way for other classes to instantiate commands that run on
    // this object
    @NotLogged
    public final RotorCommandFactory commands = new RotorCommandFactory();

    // Creates an instance of a rotor
    public Rotor()
    {
        // Inputs is just a container for variables. It can always be initialized using
        // the default constructor
        _inputs = new RotorIOInputs();

        // Instantiate an appropriate physical layer object based on if we're running on
        // a real robot or in simulation
        if (RobotBase.isReal())
        {
            _io = new RotorIOReal();
        }
        else
        {
            _io = new RotorIOSim();
        }
    }

    // Executes every 20ms. Used to measure inputs from the physical layer
    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
    }

    // Gets the current rate of rotation of the rotor motor
    public AngularVelocity getRate()
    {
        return _inputs.rotationRate;
    }

    // Sets the desired angular rate of rotation of the rotor
    public void setRate(AngularVelocity rate)
    {
        _io.setRate(rate);
    }

    // Commands the rotor to stop rotating
    public void stop()
    {
        _io.stop();
    }

    /************
     * Commands *
     ************/

    // Nested helper class responsible for creating commands that act on the
    // enclosing Rotor object instance
    public class RotorCommandFactory
    {
        // Creates a command to set the angular rate of rotation of the rotor
        public Command setRate(AngularVelocity rate)
        {
            return runOnce(() -> Rotor.this.setRate(rate));
        }

        // Creates a command to stop the rotation of the rotor
        public Command stop()
        {
            return runOnce(Rotor.this::stop);
        }

        // Creates a command to set the angular rate of rotation of the rotor. When the
        // command ends, the rotor stops rotating
        public Command run(AngularVelocity rate)
        {
            return startEnd(() -> Rotor.this.setRate(rate), Rotor.this::stop);
        }
    }
}
