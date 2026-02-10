package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
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
            _mode = ShootMode.Shoot;
            setState(ShooterState.Preparing);
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

    public Command getPreparePassCmd(double flywheelRpm, double hoodAngleDeg)
    {
        return runOnce(() ->
        {
            _mode             = ShootMode.Pass;
            _passFlywheelRpm  = flywheelRpm;
            _passHoodAngleDeg = hoodAngleDeg;
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

    private final ShooterFlywheel _flywheel;
    private final Hood            _hood;
    private final ShooterTurret   _turret;
    private final ShooterFeeder   _feeder;
    @Logged
    private ShooterState          _state              = ShooterState.Idle;
    @Logged
    private ShootMode             _mode               = ShootMode.Shoot;
    @Logged
    private double                _lastDistanceMeters = 0.0;
    @Logged
    private double                _passFlywheelRpm    = 0.0;
    @Logged
    private double                _passHoodAngleDeg   = 0.0;

    public Shooter()
    {
        this(new Hood());
    }

    public Shooter(Hood hood)
    {
        _flywheel = new ShooterFlywheel();
        _hood     = hood;
        _turret   = new ShooterTurret();
        _feeder   = new ShooterFeeder();
    }

    @Override
    public void periodic()
    {
        _flywheel.periodic();
        _turret.periodic();
        _feeder.periodic();

        updateState();
        applySetpoints();
    }

    @Override
    public void simulationPeriodic()
    {
        _flywheel.simulationPeriodic();
        _turret.simulationPeriodic();
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
            // _hood.stop();
            _turret.setState(ShooterTurret.TurretState.Off);
            _feeder.set(false);
            return;
        }

        if (_mode == ShootMode.Shoot)
        {
            _turret.setState(ShooterTurret.TurretState.Track);
            if (_turret.hasTarget())
            {
                _lastDistanceMeters = _turret.getDistanceToTarget();
            }

            double distance = _lastDistanceMeters;
            _flywheel.setVelocity(ShooterConstants.getFlywheelSpeedForDistance(distance));
            _hood.setHoodPosition(HoodPosition.Shoot);
        }
        else
        {
            _turret.setState(ShooterTurret.TurretState.Pass);
            _flywheel.setVelocity(_passFlywheelRpm);
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
        return _turret.atSetpoint();
    }

    public double getDistanceToTarget()
    {
        return _turret.getDistanceToTarget();
    }

    private boolean isReadyInternal()
    {
        return _flywheel.atSpeed() && _hood.atSetpoint() && _turret.atSetpoint();
    }
}
