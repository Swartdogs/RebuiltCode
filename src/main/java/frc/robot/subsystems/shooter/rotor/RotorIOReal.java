package frc.robot.subsystems.shooter.rotor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

/*
 * A real implementation of the physical layer (motors and sensors) of the rotor
 * sub-component of the shooter subsystem.
 */
public class RotorIOReal implements RotorIO
{
    // The rotor motor
    private final TalonFX _rotorMotor;

    // The request object for running the rotor at a specified velocity
    private final VelocityVoltage _velocityRequest;

    // Constructs a RotorIOReal instance
    public RotorIOReal()
    {
        // Initialize the motor
        _rotorMotor = new TalonFX(CANConstants.ROTOR_MOTOR);

        // Set up the velocity request for rotating at a desired rate
        _velocityRequest = new VelocityVoltage(RPM.zero()).withSlot(0);

        // Start configuring the motor
        var config = new TalonFXConfiguration();

        // Motor configurations related to current limits
        config.CurrentLimits.StatorCurrentLimit       = ShooterConstants.ROTOR_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Motor configurations related to general motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive;

        // Motor configurations related to closed loop gains
        config.Slot0.kG = ShooterConstants.ROTOR_KG;
        config.Slot0.kS = ShooterConstants.ROTOR_KS;
        config.Slot0.kV = ShooterConstants.ROTOR_KV;
        config.Slot0.kA = ShooterConstants.ROTOR_KA;
        config.Slot0.kP = ShooterConstants.ROTOR_KP;
        config.Slot0.kI = ShooterConstants.ROTOR_KI;
        config.Slot0.kD = ShooterConstants.ROTOR_KD;

        // Set the gear ratio of the motor
        config.Feedback.SensorToMechanismRatio = ShooterConstants.ROTOR_GEAR_RATIO.in(Value);

        // Apply the configuration to the motor
        _rotorMotor.getConfigurator().apply(config);
    }

    // Allows the RotorIOReal object to read sensor inputs. Updated
    // values are loaded into the provided "inputs" parameter.
    public void updateInputs(RotorIOInputs inputs)
    {
        // Save off the new inputs as measured by the motor
        inputs.appliedVoltage = _rotorMotor.getMotorVoltage().getValue();
        inputs.currentDraw    = _rotorMotor.getStatorCurrent().getValue();
        inputs.rotationRate   = _rotorMotor.getVelocity().getValue();
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

        _rotorMotor.setControl(_velocityRequest.withVelocity(rate));
    }

    // Commands the rotor to stop rotating
    public void stop()
    {
        _rotorMotor.setVoltage(0);
    }
}
