package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Hood.HoodPosition;

import frc.robot.util.Utilities;
import frc.robot.Constants;
import frc.robot.util.GameState;

@Logged
public class Shooter extends SubsystemBase
{
    public enum ShooterState
    {
        Idle, HubInactive, Tracking, Ready
    }

    public enum ShootMode
    {
        Shoot, Pass
    }

    public Command getAimCmd()
    {
        return startEnd(() ->
            double distance = _turret.getDistanceToTarget();
            _flywheel.setVelocity(Constants.Shooter.getFlywheelSpeedForDistance(distance));
            _hood.setAngle(Constants.Shooter.getHoodAngleForDistance(distance));
        }, () ->
        {
            _flywheel.stop();
            _hood.stop();
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
            _mode                 = ShootMode.Pass;
            _passFlywheelVelocity = RPM.of(flywheelRpm);
            _passHoodAngleDeg     = hoodAngleDeg;
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

    private final Flywheel      _flywheel;
    private final Hood          _hood;
    private final ShooterTurret _turret;
    private final Feeder _feeder;
    @Logged
    private ShooterState        _state                = ShooterState.Idle;
    @Logged
    private ShootMode           _mode                 = ShootMode.Shoot;
    @Logged
    private double              _lastDistanceMeters   = 0.0;
    @Logged
    private AngularVelocity     _passFlywheelVelocity = RPM.zero();
    @Logged
    private double              _passHoodAngleDeg     = 0.0;
    @Logged
    private boolean               _hasTarget = false;

    public Shooter()
    {
        _flywheel = new Flywheel();
        _hood     = new Hood();
        _turret   = new ShooterTurret();
        _feeder   = new Feeder();
    }

    public Command getStopCmd()
    {
        return runOnce(() ->
        {
            _flywheel.stop();
            _hood.stop();
        });
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
            _turret.setState(ShooterTurret.TurretState.Track);
        }
        else
        {
            _turret.setState(ShooterTurret.TurretState.Off);
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
        if (!GameState.isHubActive())
        {
            _state = ShooterState.HubInactive;
            return;
        }

        if (_flywheel.getVelocity() <= 0)
        {
            if (_turret.hasTarget())
            {
                _state = ShooterState.Tracking;
            }
            else
            {
                _state = ShooterState.Idle;
            }
        }
        else if (_flywheel.atSpeed() && _hood.atSetpoint() && _turret.atSetpoint())
        {
            _state = ShooterState.Ready;
        }
        else
        {
            _state = ShooterState.Tracking;
        }
    }

    private void applySetpoints()
    {
        if (_state == ShooterState.Idle)
        {
            _flywheel.stop();
            _hood.stop();
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

    public boolean isReady()
    {
        return _state == ShooterState.Ready;
    }

    public boolean isHubActive()
    {
        return GameState.isHubActive();
    }

    public boolean hasTarget()
    {
        return _hasTarget;
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

    public boolean atSetpoint()
    {
        return _turret.atSetpoint();
    }
}
