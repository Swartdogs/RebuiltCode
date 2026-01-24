package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterFlywheel extends SubsystemBase
{
    private final SparkFlex                 _leadMotor;
    private final SparkFlex                 _followMotor;
    private final SparkClosedLoopController _closedLoopController;
    private final RelativeEncoder           _encoder;
    private double                          _velocity       = 0.0;
    private double                          _velocityTarget = 0.0;

    public ShooterFlywheel()
    {
        if (RobotBase.isSimulation())
        {
            _leadMotor            = null;
            _followMotor          = null;
            _closedLoopController = null;
            _encoder              = null;
            return;
        }

        _leadMotor   = new SparkFlex(Constants.CAN.FLYWHEEL_LEAD, MotorType.kBrushless);
        _followMotor = new SparkFlex(Constants.CAN.FLYWHEEL_FOLLOW, MotorType.kBrushless);

        _leadMotor.setCANTimeout(250);
        _followMotor.setCANTimeout(250);

        var leaderConfig = new SparkFlexConfig();
        leaderConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(Constants.Shooter.FLYWHEEL_CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        leaderConfig.closedLoop.p(Constants.Shooter.FLYWHEEL_KP).d(Constants.Shooter.FLYWHEEL_KD);

        leaderConfig.closedLoop.feedForward.kV(Constants.Shooter.FLYWHEEL_KV);

        _leadMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var followerConfig = new SparkFlexConfig();
        followerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(Constants.Shooter.FLYWHEEL_CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE).follow(_leadMotor, true);

        _followMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _closedLoopController = _leadMotor.getClosedLoopController();
        _encoder              = _leadMotor.getEncoder();

        _leadMotor.setCANTimeout(0);
        _followMotor.setCANTimeout(0);
    }

    @Override
    public void periodic()
    {
        if (_encoder == null) return;

        _velocity = _encoder.getVelocity();
    }

    public void setVelocity(double rpm)
    {
        if (_closedLoopController == null) return;

        _velocityTarget = rpm;
        _closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void stop()
    {
        _velocityTarget = 0.0;

        if (_leadMotor != null)
        {
            _leadMotor.setVoltage(0);
        }
    }

    public boolean atSpeed()
    {
        if (_velocityTarget <= 0) return true;

        double error = Math.abs(_velocity - _velocityTarget);
        return error <= _velocityTarget * Constants.Shooter.VELOCITY_RANGE;
    }

    public double getVelocity()
    {
        return _velocity;
    }
}
