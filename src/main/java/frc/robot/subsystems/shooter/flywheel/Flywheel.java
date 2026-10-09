package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOInputs;

/*
 * Class containing the business logic for how the flywheel component of the
 * shooter subsystem works
 */
@Logged
public class Flywheel extends SubsystemBase
{
    // The inputs measured from the flywheel
    @Logged
    private final FlywheelIOInputs _inputs;

    // The physical layer (motors and sensors) the business logic interacts with
    @NotLogged
    private final FlywheelIO _io;

    // Provides a clean way for other classes to instantiate commands that run on
    // this object
    @NotLogged
    public final FlywheelCommandFactory commands = new FlywheelCommandFactory();

    // The target rate of rotation for the flywheel
    @Logged
    private AngularVelocity _targetRate;

    // Creates an instance of a flywheel
    public Flywheel()
    {
        // Inputs is just a container for variables. In can always be initialized using
        // the default constructor
        _inputs = new FlywheelIOInputs();

        // Instantiate an appropriate physical layer object based on if we're running on
        // a real robot or in simulation
        if (RobotBase.isReal())
        {
            _io = new FlywheelIOReal();
        }
        else
        {
            _io = new FlywheelIOSim();
        }

        // Initialize private non-final member variables
        _targetRate = RPM.zero();
    }

    // Executes every 20ms. Used to measure inputs from the physical layer
    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
    }

    // Gets the current rate of rotation of the flywheel
    public AngularVelocity getRate()
    {
        // Since the flywheel is made up of two motors, we'll take the average of both
        // motors' speeds. The difference should be negligible since they're
        // mechanically linked on the same output shaft.
        return _inputs.leftMotorRotationRate.plus(_inputs.rightMotorRotationRate).div(2);
    }

    // Gets the current target rate of rotation for the flywheel
    public AngularVelocity getTargetRate()
    {
        return _targetRate;
    }

    // Determines if the flywheel is currently spinning at the target angular
    // velocity. Only returns true if the flywheel is actually on.
    public boolean atSpeed()
    {
        return _targetRate.gt(RPM.zero()) && getRate().isNear(_targetRate, ShooterConstants.FLYWHEEL_TOLERANCE);
    }

    // Sets the target rate of rotation for the flywheel
    public void setRate(AngularVelocity rate)
    {
        if (rate.lte(RPM.zero()))
        {
            stop();
            return;
        }

        _targetRate = rate;
        _io.setRate(_targetRate);
    }

    // Commands the flywheel to stop rotating
    public void stop()
    {
        _targetRate = RPM.zero();
        _io.stop();
    }

    /************
     * Commands *
     ************/

    // Nested helper class responsible for creating commands that act on the
    // enclosing Flywheel object instance
    public class FlywheelCommandFactory
    {
        // Creates a command to set the angular rate of rotation of the flywheel
        public Command setRate(AngularVelocity rate)
        {
            return runOnce(() -> Flywheel.this.setRate(rate));
        }

        // Creates a command to stop the rotation of the flywheel
        public Command stop()
        {
            return runOnce(Flywheel.this::stop);
        }

        // Creates a command to set the angular rate of rotation of the flywheel. When
        // the command ends, the flywheel stops rotating
        public Command run(AngularVelocity rate)
        {
            return startEnd(() -> Flywheel.this.setRate(rate), Flywheel.this::stop);
        }
    }
}
