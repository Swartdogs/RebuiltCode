package frc.robot.subsystems.shooter.rotor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

/*
 * A simulated implementation of the physical layer (motors and sensors) of the
 * rotor sub-component of the shooter subsystem.
 */
public class RotorIOSim implements RotorIO
{
    // The mathematical model of the rotor motor
    private final DCMotor _rotorModel;

    // The simulation object for the rotor. The rotor is essentially a sideways
    // flywheel
    private final FlywheelSim _rotorSim;

    // The feed forward model for velocity control
    private final SimpleMotorFeedforward _feedForward;

    // The pid controller for velocity control
    private final PIDController _pid;

    // The target velocity
    private AngularVelocity _targetVelocity;

    // Constructs a RotorIOSim instance
    public RotorIOSim()
    {
        // Initialize the mathematical model
        _rotorModel = DCMotor.getKrakenX60(1);

        // Initialize the simulation object
        _rotorSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(_rotorModel, 0.1, ShooterConstants.ROTOR_GEAR_RATIO.in(Value)), _rotorModel);

        // Initialize feed forward for velocity control
        _feedForward = new SimpleMotorFeedforward(ShooterConstants.ROTOR_KS.in(Volts), ShooterConstants.ROTOR_KV.in(Volts.per(RPM)), ShooterConstants.ROTOR_KA.in(Volts.per(RotationsPerSecondPerSecond)));

        // Initialize pid for velocity control
        _pid = new PIDController(ShooterConstants.ROTOR_KP.in(Volts.per(RPM)), 0.0, ShooterConstants.ROTOR_KD.in(Volts.per(RPM.per(Second))));

        // Initialize target velocity. Off by default
        _targetVelocity = RPM.zero();
    }

    // Allows the RotorIOReal object to read sensor inputs. Updated
    // values are loaded into the provided "inputs" parameter.
    public void updateInputs(RotorIOInputs inputs)
    {
        // Calculate the voltage to apply to the rotor
        var volts = Volts.zero();

        // Only calculate the feedforward and feedback if our target velocity is above
        // zero
        if (_targetVelocity.gt(RPM.zero()))
        {
            var feedForward = Volts.of(_feedForward.calculate(_targetVelocity.in(RPM)));
            var feedback    = Volts.of(_pid.calculate(_rotorSim.getAngularVelocityRPM(), _targetVelocity.in(RPM)));

            volts = feedForward.plus(feedback);
        }

        // Set the input voltage for the model and calculate the new inputs
        _rotorSim.setInputVoltage(volts.in(Volts));
        _rotorSim.update(GeneralConstants.LOOP_PERIOD.in(Seconds));

        // Save off the new inputs
        inputs.appliedVoltage = Volts.of(_rotorSim.getInputVoltage());
        inputs.currentDraw    = Amps.of(_rotorSim.getCurrentDrawAmps());
        inputs.rotationRate   = _rotorSim.getAngularVelocity();
    }

    // Sets the desired angular rate of rotation of the rotor
    public void setRate(AngularVelocity rate)
    {
        // If a non-positive speed is requested, turn off the rotor
        if (rate.lte(RPM.zero()))
        {
            stop();
            return;
        }

        _targetVelocity = rate;
    }

    // Commands the rotor to stop rotating
    public void stop()
    {
        _targetVelocity = RPM.zero();
    }
}
