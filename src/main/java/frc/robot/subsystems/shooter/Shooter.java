package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Shooter extends SubsystemBase
{
    public enum ShooterState
    {
        Idle, Preparing, Ready, Firing
    }

    public enum ShootMode
    {
        Shoot, Pass
    }

    private final Flywheel  _flywheel;
    private final Feeder    _feeder;
    private ShooterState    _state;
    private AngularVelocity _targetVelocity;

    public Shooter()
    {
        _flywheel       = new Flywheel();
        _feeder         = new Feeder();
        _state          = ShooterState.Idle;
        _targetVelocity = RPM.zero();
    }

    @Override
    public void periodic()
    {
        switch (_state)
        {
            case Preparing:
                _feeder.set(false);
                _flywheel.setVelocity(_targetVelocity);
                if (_flywheel.atSpeed())
                {
                    _state = ShooterState.Ready;
                }
                break;

            case Ready:
                _feeder.set(false);
                _flywheel.setVelocity(_targetVelocity);
                if (!_flywheel.atSpeed())
                {
                    _state = ShooterState.Preparing;
                }
                break;

            case Firing:
                _flywheel.setVelocity(_targetVelocity);
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
    }

    @Override
    public void simulationPeriodic()
    {
        // Feeder has no simulation periodic
        // _feeder.simulationPeriodic();
        _flywheel.simulationPeriodic();
    }

    public void start(AngularVelocity velocity)
    {
        if (_state == ShooterState.Idle)
        {
            _state = ShooterState.Preparing;
        }
        if (_state != ShooterState.Firing)
        {
            _targetVelocity = velocity;
        }
    }

    public void stop()
    {
        _state          = ShooterState.Idle;
        _targetVelocity = RPM.of(0);
    }

    public void fire()
    {
        if (_state == ShooterState.Ready)
        {
            _state = ShooterState.Firing;
        }
    }

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
}
