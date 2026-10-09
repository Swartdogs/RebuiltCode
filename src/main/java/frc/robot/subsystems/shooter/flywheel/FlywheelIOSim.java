package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
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
 */
public class FlywheelIOSim implements FlywheelIO
{
    // The mathematical model of the flywheel motors.
    private final DCMotor _flywheelModel;

    // The simulation object for the flywheel.
    private final FlywheelSim _flywheelSim;

    // The feed forward model for velocity control.
    private final SimpleMotorFeedforward _feedForward;

    // The pid controller for velocity control.
    private final PIDController _pid;

    // The target velocity.
    private AngularVelocity _targetVelocity;

    // Constructs a FlywheelIOSim instance.
    public FlywheelIOSim()
    {
        // Initialize the mathematical model
        _flywheelModel = DCMotor.getNeoVortex(2);

        // Initialize the simulation object
        _flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(_flywheelModel, 0.001, 1), _flywheelModel);

        // Initialize feed forward for velocity control
        _feedForward = new SimpleMotorFeedforward(ShooterConstants.FLYWHEEL_KS.in(Volts), ShooterConstants.FLYWHEEL_KV.in(Volts.per(RPM)), ShooterConstants.FLYWHEEL_KA.in(Volts.per(RotationsPerSecondPerSecond)));

        // Initialize the pid for velocity control
        _pid = new PIDController(ShooterConstants.FLYWHEEL_KP.in(Volts.per(RPM)), 0.0, ShooterConstants.FLYWHEEL_KD.in(Volts.per(RPM.per(Second))));

        // Initialize target velocity. Off by default
        _targetVelocity = RPM.zero();
    }

    // Allows the FlywheelIOSim object to read sensor inputs. Updated values are
    // loaded into the provided "inputs" parameter.
    @Override
    public void updateInputs(FlywheelIOInputs inputs)
    {
        // Calculate the voltage to apply to the flywheel
        var volts = Volts.zero();

        // Only calculate the feedforward and feedback if our target velocity is above
        // zero
        if (_targetVelocity.gt(RPM.zero()))
        {
            var feedForward = Volts.of(_feedForward.calculate(_targetVelocity.in(RPM)));
            var feedback    = Volts.of(_pid.calculate(_flywheelSim.getAngularVelocityRPM(), _targetVelocity.in(RPM)));

            volts = feedForward.plus(feedback);
        }

        // Set the input voltage for the model and calculate the new inputs
        _flywheelSim.setInputVoltage(volts.in(Volts));
        _flywheelSim.update(GeneralConstants.LOOP_PERIOD.in(Seconds));

        // Save off the new inputs
        inputs.leftMotorAppliedVoltage = Volts.of(_flywheelSim.getInputVoltage());
        inputs.leftMotorCurrentDraw    = Amps.of(_flywheelSim.getCurrentDrawAmps());
        inputs.leftMotorRotationRate   = _flywheelSim.getAngularVelocity();

        // The flywheel plant model measures both motors together. For simulation, it's
        // fine to assume that both motors are doing the exact same thing.
        inputs.rightMotorAppliedVoltage = inputs.leftMotorAppliedVoltage;
        inputs.rightMotorCurrentDraw    = inputs.leftMotorCurrentDraw;
        inputs.rightMotorRotationRate   = inputs.leftMotorRotationRate;
    }

    // Sets the desired angular rate of rotation of the flywheel.
    @Override
    public void setRate(AngularVelocity rate)
    {
        _targetVelocity = rate;
    }

    // Commands the flywheel to stop rotating
    @Override
    public void stop()
    {
        _targetVelocity = RPM.zero();
    }
}
