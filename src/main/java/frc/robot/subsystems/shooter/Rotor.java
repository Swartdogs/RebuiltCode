package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Rotor
{
    private final SparkFlex _rotorMotor;
    @Logged
    private Voltage         _rotorMotorVoltage = Volts.zero();
    @Logged
    private boolean         _enabled           = false;

    public Rotor()
    {
        _rotorMotor = new SparkFlex(CANConstants.ROTOR_MOTOR, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit((int)ShooterConstants.ROTOR_CURRENT_LIMIT.in(Amps)).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        _rotorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic()
    {
        _rotorMotorVoltage = Volts.of(_rotorMotor.getAppliedOutput() * _rotorMotor.getBusVoltage());
    }

    public void set(boolean on)
    {
        _enabled = on;
        var targetVoltage = on ? ShooterConstants.ROTOR_VOLTAGE : Volts.zero();
        _rotorMotor.setVoltage(targetVoltage.in(Volts));
    }
}
