package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import frc.robot.Constants;

@Logged
public class ShooterFeeder {
    private final SparkFlex _feederMotor;
    @Logged
    private double _motorVoltage = 0.0;

    public ShooterFeeder() {
        _feederMotor = new SparkFlex(0, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.Shooter.FEEDER_CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        _feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void periodic()
    {
        _motorVoltage = _feederMotor.getAppliedOutput() * _feederMotor.getBusVoltage();
    }
    public void set(boolean on) {
        _feederMotor.setVoltage(on? Constants.Shooter.FEEDER_VOLTAGE: 0);
    }
}
