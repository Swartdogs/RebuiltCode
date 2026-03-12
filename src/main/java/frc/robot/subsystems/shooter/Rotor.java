package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.MotorHook;
import frc.robot.TestHook;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Rotor
{
    private final SparkFlex _rotorMotor;
    @Logged
    private Voltage         _rotorMotorVoltage = Volts.of(0.0);

    public Rotor()
    {
        _rotorMotor = new SparkFlex(CANConstants.ROTOR_MOTOR, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit((int)ShooterConstants.ROTOR_CURRENT_LIMIT.in(Amps)).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        _rotorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic()
    {
        _rotorMotorVoltage = Volts.of(_rotorMotor.getAppliedOutput() * _rotorMotor.getBusVoltage());
    }

    public void set(boolean on)
    {
        Voltage targetVoltage = on ? ShooterConstants.ROTOR_VOLTAGE : Volts.zero();
        _rotorMotor.setVoltage(targetVoltage.in(Volts));
    }

    private class FeederHook extends MotorHook
    {
        @Override
        public void stop()
        {
            set(false);
        }

        @Override
        public void setRate(double rate)
        {
            _rotorMotor.setVoltage(GeneralConstants.MOTOR_VOLTAGE.times(rate * _polarity));
        }
    }
}
