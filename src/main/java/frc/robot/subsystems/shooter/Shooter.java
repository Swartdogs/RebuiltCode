package frc.robot.subsystems.shooter;

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
    private final Flywheel _flywheel;
    private final Feeder   _feeder;
    private final Hood     _hood;
    private ShooterState   _state;

    public Shooter()
    {
        _flywheel = new Flywheel();
        _feeder   = new Feeder();
        _hood     = new Hood();
        _state    = ShooterState.Idle;

        _hood.setHoodPosition(HoodPosition.Shoot);
        _feeder.set(false);
        _flywheel.stop();
    }

    @Override
    public void periodic()
    {
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
        if (_state != ShooterState.Firing)
        {
            _hood.setHoodPosition(HoodPosition.Pass);
        }

        _flywheel.setVelocity(ShooterConstants.PASS_FLYWHEEL_VELOCITY);
        _state = ShooterState.Priming;
    }
}
