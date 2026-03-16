package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Rotor
{
    private final TalonFX _rotorMotor;
    @Logged
    private Voltage       _rotorMotorVoltage = Volts.zero();
    @Logged
    private boolean       _enabled           = false;

    public Rotor()
    {
        _rotorMotor = new TalonFX(CANConstants.ROTOR_MOTOR);

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.ROTOR_CURRENT_LIMIT.in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.Clockwise_Positive;
        _rotorMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig));
    }

    public void periodic()
    {
        _rotorMotorVoltage = _rotorMotor.getMotorVoltage().getValue();
    }

    public void set(boolean on)
    {
        _enabled = on;
        var targetVoltage = on ? ShooterConstants.ROTOR_VOLTAGE : Volts.zero();
        _rotorMotor.setVoltage(targetVoltage.in(Volts));
    }

    public void stop()
    {
        set(false);
    }
}
