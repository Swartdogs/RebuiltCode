package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class IntakeIOSparkFlex implements IntakeIO
{
    private final SparkFlex _motor;

    public IntakeIOSparkFlex()
    {
        _motor = new SparkFlex(Constants.CAN.INTAKE, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Intake.CURRENT_LIMIT)
        .voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        _motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        inputs.motorVolts   = _motor.getAppliedOutput() * _motor.getBusVoltage();
    }

    @Override
    public void setVolts(double volts)
    {
        _motor.setVoltage(volts);
    }
}