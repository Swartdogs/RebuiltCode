package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Feeder
{
    private final SparkFlex _feederMotor;
    private Voltage         _feederMotorVoltage = Volts.of(0.0);

    public Feeder()
    {
        _feederMotor = new SparkFlex(CANConstants.FEEDER_MOTOR, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.FEEDER_CURRENT_LIMIT).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE);

        _feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic()
    {
        _feederMotorVoltage = Volts.of(_feederMotor.getAppliedOutput() * _feederMotor.getBusVoltage());
    }

    public void set(boolean on)
    {
        _feederMotor.setVoltage(on ? ShooterConstants.FEEDER_VOLTAGE : 0.0);
    }
}
