package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterFlywheel extends SubsystemBase
{
    private final SparkFlex                 _leadMotor;
    private final SparkFlex                 _followMotor;
    private final SparkClosedLoopController _closedLoopController;
    private final RelativeEncoder           _encoder;
    private final SparkSim                  _leadMotorSim;
    private double                          _velocity       = 0.0;
    private double                          _velocityTarget = 0.0;

    public ShooterFlywheel()
    {
        _leadMotor   = new SparkFlex(Constants.CAN.FLYWHEEL_LEAD, MotorType.kBrushless);
        _followMotor = new SparkFlex(Constants.CAN.FLYWHEEL_FOLLOW, MotorType.kBrushless);

        if (RobotBase.isReal())
        {
            _leadMotor.setCANTimeout(250);
            _followMotor.setCANTimeout(250);
        }

        var config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(Constants.Shooter.FLYWHEEL_CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        config.inverted(false);
        config.closedLoop.p(Constants.Shooter.FLYWHEEL_KP).d(Constants.Shooter.FLYWHEEL_KD);
        config.closedLoop.feedForward.kS(Constants.Shooter.FLYWHEEL_KS).kV(Constants.Shooter.FLYWHEEL_KV).kA(Constants.Shooter.FLYWHEEL_KA);
        _leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(_leadMotor, true);
        _followMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _closedLoopController = _leadMotor.getClosedLoopController();
        _encoder              = _leadMotor.getEncoder();

        if (RobotBase.isReal())
        {
            _leadMotor.setCANTimeout(0);
            _followMotor.setCANTimeout(0);
            _leadMotorSim = null;
        }
        else
        {
            _leadMotorSim = new SparkSim(_leadMotor, DCMotor.getNeoVortex(2));
        }
    }

    @Override
    public void periodic()
    {
        _velocity = _encoder.getVelocity();
    }

    @Override
    public void simulationPeriodic()
    {
        _leadMotorSim.iterate(getVelocity(), RoboRioSim.getVInVoltage(), Constants.General.LOOP_PERIOD_SECS);
    }

    public void setVelocity(double rpm)
    {
        _velocityTarget = rpm;
        _closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void stop()
    {
        _velocityTarget = 0.0;
        _leadMotor.setVoltage(0);
    }

    public boolean atSpeed()
    {
        if (_velocityTarget <= 0) return true;

        double error = Math.abs(getVelocity() - _velocityTarget);
        return error <= _velocityTarget * Constants.Shooter.FLYWHEEL_TOLERANCE;
    }

    public double getVelocity()
    {
        return _velocity;
    }
}
