package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
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
            if (_state != ShooterState.Firing)
            {
                _hood.setHoodPosition(mode);
            }
        });
    }

    public Command passCmd()
    {
        return startEnd(this::pass, this::stop);
    }

    /*************
     * SUBSYSTEM *
     *************/
    private final Flywheel  _flywheel;
    private final Feeder    _feeder;
    private final Hood      _hood;
    private ShooterState    _state;
    private DoubleSupplier  _velocityAdjustmentFunc;
    private AngularVelocity _targetVelocity;

    public Shooter(DoubleSupplier velocityAdjustmentFunc)
    {
        _flywheel               = new Flywheel();
        _feeder                 = new Feeder();
        _hood                   = new Hood();
        _state                  = ShooterState.Idle;
        _velocityAdjustmentFunc = velocityAdjustmentFunc;
        _targetVelocity         = RPM.zero();

        _hood.setHoodPosition(HoodPosition.Shoot);
        _feeder.set(false);
        _flywheel.stop();
    }

    public Shooter()
    {
        this(() -> 0.0);
    }

    @Override
    public void periodic()
    {
        switch (_state)
        {
            case Preparing:
                _flywheel.setVelocity(getDesiredVelocity());
                _feeder.set(false);
                if (_flywheel.atSpeed() && _hood.atSetpoint())
                {
                    _state = ShooterState.Ready;
                }
                break;

            case Priming:
                _flywheel.setVelocity(getDesiredVelocity());
                _feeder.set(false);
                if (_flywheel.atSpeed() && _hood.atSetpoint())
                {
                    _state = ShooterState.Firing;
                }
                break;

            case Ready:
                _flywheel.setVelocity(getDesiredVelocity());
                _feeder.set(false);
                if (!_flywheel.atSpeed())
                {
                    _state = ShooterState.Preparing;
                }
                break;

            case Firing:
                _flywheel.setVelocity(getDesiredVelocity());
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
        if (_state == ShooterState.Idle)
        {
            _state = ShooterState.Preparing;
        }

        _targetVelocity = velocity;
    }

    public void stop()
    {
        _state          = ShooterState.Idle;
        _targetVelocity = RPM.zero();
    }

    public void fire()
    {
        if (_state == ShooterState.Ready)
        {
            _state = ShooterState.Firing;
        }
    }

    private AngularVelocity getDesiredVelocity()
    {
        if (_hood.getHoodPosition() == HoodPosition.Pass)
        {
            return getPassVelocity();
        }

        return _targetVelocity;
    }

    private AngularVelocity getPassVelocity()
    {
        return ShooterConstants.PASS_FLYWHEEL_VELOCITY.plus(ShooterConstants.MAX_PASS_VELOCITY_ADJUSTMENT.times(_velocityAdjustmentFunc.getAsDouble()));
    }

    public void pass()
    {
        if (_state != ShooterState.Firing)
        {
            _hood.setHoodPosition(HoodPosition.Pass);
        }

        _state = ShooterState.Priming;
    }
}
