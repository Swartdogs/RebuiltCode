package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Hood.HoodPosition;

@Logged
public class Shooter extends SubsystemBase
{
    public enum ShooterState
    {
        Idle, Preparing, Ready, Firing, Priming;
    }

    /************
     * COMMANDS *
     ************/
    public Command fireCmd()
    {
        return runOnce(this::fire);
    }

    public Command stopCmd()
    {
        return runOnce(this::stop);
    }

    public Command startCmd(AngularVelocity velocity)
    {
        return runOnce(() -> start(velocity));
    }

    public Command setShootModeCmd(Hood.HoodPosition mode)
    {
        return runOnce(() ->
        {
            if (_state != ShooterState.Firing && !_calibrationMode)
            {
                _hood.setHoodPosition(mode);
            }
        });
    }

    public Command passCmd()
    {
        return startEnd(this::pass, this::stop);
    }

    public Command toggleCalibrationModeCmd()
    {
        return runOnce(this::toggleCalibrationMode);
    }

    public Command bumpCalibrationFlywheelCmd(AngularVelocity delta)
    {
        return runOnce(() -> bumpCalibrationFlywheel(delta));
    }

    public Command bumpCalibrationHoodCmd(Angle delta)
    {
        return runOnce(() -> bumpCalibrationHood(delta));
    }

    public Command captureCalibrationSampleCmd(Supplier<Distance> distanceSupplier)
    {
        return runOnce(() -> logCalibrationSample(distanceSupplier == null ? Meters.zero() : distanceSupplier.get()));
    }

    /*************
     * SUBSYSTEM *
     *************/
    private final Flywheel  _flywheel;
    private final Feeder    _feeder;
    private final Hood      _hood;
    @Logged
    private boolean         _calibrationMode;
    @Logged
    private AngularVelocity _calibrationFlywheelVelocity;
    @Logged
    private Angle           _calibrationHoodAngle;
    private ShooterState    _state;

    public Shooter()
    {
        _flywheel                    = new Flywheel();
        _feeder                      = new Feeder();
        _hood                        = new Hood();
        _state                       = ShooterState.Idle;
        _calibrationMode             = false;
        _calibrationFlywheelVelocity = ShooterConstants.CALIBRATION_DEFAULT_FLYWHEEL_VELOCITY;
        _calibrationHoodAngle        = ShooterConstants.HOOD_SHOOT_ANGLE;

        _hood.setHoodPosition(HoodPosition.Shoot);
        _feeder.set(false);
        _flywheel.stop();
    }

    @Override
    public void periodic()
    {
        if (_calibrationMode)
        {
            _hood.setHoodAngle(_calibrationHoodAngle);
            _flywheel.setVelocity(_calibrationFlywheelVelocity);
            if (_state == ShooterState.Idle)
            {
                _state = ShooterState.Preparing;
            }
        }

        switch (_state)
        {
            case Preparing:
                _feeder.set(false);
                if (_flywheel.atSpeed() && _hood.atSetpoint())
                {
                    _state = ShooterState.Ready;
                }
                break;

            case Priming:
                _feeder.set(false);
                if (_flywheel.atSpeed() && _hood.atSetpoint())
                {
                    _state = ShooterState.Firing;
                }
                break;

            case Ready:
                _feeder.set(false);
                if (!_flywheel.atSpeed())
                {
                    _state = ShooterState.Preparing;
                }
                break;

            case Firing:
                _feeder.set(true);
                break;
            case Idle:
            default:
                _flywheel.stop();
                _feeder.set(false);
                break;
        }
        _flywheel.periodic();
        _feeder.periodic();
        _hood.periodic();
    }

    @Override
    public void simulationPeriodic()
    {
        _flywheel.simulationPeriodic();
        _hood.simulationPeriodic();
    }

    public void start(AngularVelocity velocity)
    {
        if (_calibrationMode)
        {
            _calibrationFlywheelVelocity = velocity;
            _hood.setHoodAngle(_calibrationHoodAngle);
        }

        if (_state == ShooterState.Idle)
        {
            _state = ShooterState.Preparing;
        }

        _flywheel.setVelocity(velocity);
    }

    public void stop()
    {
        _state = ShooterState.Idle;
        _flywheel.stop();
        _feeder.set(false);
    }

    public void fire()
    {
        if (_state == ShooterState.Ready)
        {
            _state = ShooterState.Firing;
        }
    }

    public void pass()
    {
        if (_calibrationMode)
        {
            return;
        }

        if (_state != ShooterState.Firing)
        {
            _hood.setHoodPosition(HoodPosition.Pass);
        }

        _flywheel.setVelocity(ShooterConstants.PASS_FLYWHEEL_VELOCITY);
        _state = ShooterState.Priming;
    }

    public boolean isCalibrationMode()
    {
        return _calibrationMode;
    }

    public AngularVelocity getCalibrationFlywheelVelocity()
    {
        return _calibrationFlywheelVelocity;
    }

    public Angle getCalibrationHoodAngle()
    {
        return _calibrationHoodAngle;
    }

    public void toggleCalibrationMode()
    {
        _calibrationMode = !_calibrationMode;
        if (_calibrationMode)
        {
            if (_flywheel.getTargetVelocity().lte(RPM.zero()))
            {
                _calibrationFlywheelVelocity = ShooterConstants.CALIBRATION_DEFAULT_FLYWHEEL_VELOCITY;
            }
            else
            {
                _calibrationFlywheelVelocity = _flywheel.getTargetVelocity();
            }

            _calibrationHoodAngle = _hood.hasSetpoint() ? _hood.getHoodSetpoint() : ShooterConstants.HOOD_SHOOT_ANGLE;
            start(_calibrationFlywheelVelocity);
            _hood.setHoodAngle(_calibrationHoodAngle);
        }
        else
        {
            stop();
            _hood.setHoodPosition(HoodPosition.Shoot);
        }
    }

    public void bumpCalibrationFlywheel(AngularVelocity delta)
    {
        if (!_calibrationMode)
        {
            return;
        }

        _calibrationFlywheelVelocity = _calibrationFlywheelVelocity.plus(delta);
        if (_calibrationFlywheelVelocity.lt(RPM.zero()))
        {
            _calibrationFlywheelVelocity = RPM.zero();
        }

        start(_calibrationFlywheelVelocity);
    }

    public void bumpCalibrationHood(Angle delta)
    {
        if (!_calibrationMode)
        {
            return;
        }

        _calibrationHoodAngle = _calibrationHoodAngle.plus(delta);
        _hood.setHoodAngle(_calibrationHoodAngle);
    }

    public void logCalibrationSample(Distance measuredDistance)
    {
        Angle           hoodTarget     = _calibrationMode ? _calibrationHoodAngle : _hood.getHoodSetpoint();
        AngularVelocity flywheelTarget = _calibrationMode ? _calibrationFlywheelVelocity : _flywheel.getTargetVelocity();
        String          message        = String.format("CALIBRATION_SAMPLE distance_m=%.3f flywheel_rpm=%.1f hood_deg=%.2f", measuredDistance.in(Meters), flywheelTarget.in(RPM), hoodTarget.in(Degrees));
        System.out.println(message);
    }
}
