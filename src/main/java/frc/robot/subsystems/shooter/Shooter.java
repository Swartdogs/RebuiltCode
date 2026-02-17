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
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Hood.HoodPosition;

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

    public Command getAimCmd()
    {
        return startEnd(() ->
        {
            var distance = _turret.getDistanceToTarget();
            _flywheel.setVelocity(ShooterConstants.getFlywheelSpeedForDistance(distance));
            _hood.setHoodPosition(Hood.HoodPosition.Shoot);
        }, () ->
        {
            setState(ShooterState.Idle);
        });
    }

    public Command getPrepareCmd()
    {
        return runOnce(() ->
        {
            _mode = ShootMode.Shoot;
            setState(ShooterState.Preparing);
        });
    }

    public Command getPreparePassCmd(AngularVelocity flywheelVelocity, Angle hoodAngle)
    {
        return runOnce(() ->
        {
            _mode                 = ShootMode.Pass;
            _passFlywheelVelocity = flywheelVelocity;
            _passHoodAngle        = hoodAngle;
            setState(ShooterState.Preparing);
        });
    }

    public Command getFireCmd()
    {
        return runOnce(() -> setState(ShooterState.Firing));
    }

    public Command getStopCmd()
    {
        return runOnce(() -> setState(ShooterState.Idle));
    }

    private final Flywheel  _flywheel;
    private final Hood      _hood;
    private final Turret    _turret;
    private final Feeder    _feeder;
    @Logged
    private ShooterState    _state                = ShooterState.Idle;
    @Logged
    private ShootMode       _mode                 = ShootMode.Shoot;
    @Logged
    private Distance        _lastDistance         = Meters.of(0.0);
    @Logged
    private AngularVelocity _passFlywheelVelocity = RPM.zero();
    @Logged
    private Angle           _passHoodAngle        = Degrees.of(0.0);
    @Logged
    private boolean         _hasTarget            = false;

    public Shooter()
    {
        _flywheel = new Flywheel();
        _hood     = new Hood();
        _turret   = new Turret(null);
        _feeder   = new Feeder();
    }

    @Override
    public void periodic()
    {
        _hood.periodic();
        _flywheel.periodic();
        _turret.periodic();
        _feeder.periodic();

        updateState();
        applySetpoints();

        if (_turret.hasTarget())
        {
            _turret.setTurretState(Turret.TurretState.Track);
        }
        else
        {
            _turret.setTurretState(Turret.TurretState.Idle);
        }

        _hasTarget = _turret.hasTarget();
    }

    @Override
    public void simulationPeriodic()
    {
        _hood.simulationPeriodic();
        _flywheel.simulationPeriodic();
        _turret.simulationPeriodic();
        _feeder.simulationPeriodic();
    }

    private void updateState()
    {
        switch (_state)
        {
            case Preparing:
                if (isReadyInternal())
                {
                    _state = ShooterState.Ready;
                }
                break;

            case Ready:
                if (!isReadyInternal())
                {
                    _state = ShooterState.Preparing;
                }
                break;

            default:
                break;
        }
    }

    private void applySetpoints()
    {
        if (_state == ShooterState.Idle)
        {
            _flywheel.stop();
            _hood.stop();
            _turret.setTurretState(Turret.TurretState.Idle);
            _feeder.set(false);
            return;
        }

        if (_mode == ShootMode.Shoot)
        {
            _turret.setTurretState(Turret.TurretState.Track);
            if (_turret.hasTarget())
            {
                _lastDistance = _turret.getDistanceToTarget();
            }

            _flywheel.setVelocity(ShooterConstants.getFlywheelSpeedForDistance(_lastDistance));
            _hood.setHoodPosition(HoodPosition.Shoot);
        }
        else
        {
            _turret.setTurretState(Turret.TurretState.Pass);
            _flywheel.setVelocity(_passFlywheelVelocity);
            _hood.setHoodPosition(HoodPosition.Pass);
        }

        _feeder.set(_state == ShooterState.Firing);
    }

    public void setState(ShooterState state)
    {
        switch (_state)
        {
            case Idle:
                if (state == ShooterState.Preparing)
                {
                    _state = state;
                }
                break;

            case Preparing:
                if (state == ShooterState.Idle)
                {
                    _state = state;
                }
                break;

            case Ready:
                if (state == ShooterState.Firing || state == ShooterState.Idle)
                {
                    _state = state;
                }
                break;

            case Firing:
                if (state == ShooterState.Idle)
                {
                    _state = state;
                }
                break;

            default:
                break;
        }
    }

    public ShooterState getState()
    {
        return _state;
    }

    public boolean isFlywheelAtSpeed()
    {
        return _flywheel.atSpeed();
    }

    public boolean isHoodAtSetpoint()
    {
        return _hood.atSetpoint();
    }

    public boolean isTurretAtSetpoint()
    {
        return _turret.isLinedUp();
    }

    public Distance getDistanceToTarget()
    {
        return _turret.getDistanceToTarget();
    }

    private boolean isReadyInternal()
    {
        return _flywheel.atSpeed() && _hood.atSetpoint() && _turret.isLinedUp();
    }

    public boolean atSetpoint()
    {
        return _flywheel.atSpeed() && _hood.atSetpoint() && _turret.isLinedUp();
    }
}
