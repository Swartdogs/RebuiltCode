package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.GameState;

@Logged
public class Shooter extends SubsystemBase
{
    public enum ShooterState
    {
        Idle, HubInactive, Tracking, Ready
    }

    public Command getAimCmd()
    {
        return startEnd(() ->
        {
            double distance = _turret.getDistanceToTarget();
            _flywheel.setVelocity(Constants.Shooter.getFlywheelSpeedForDistance(distance));
            _hood.setAngle(Constants.Shooter.getHoodAngleForDistance(distance));
        }, () ->
        {
            _flywheel.stop();
            _hood.stop();
        });
    }

    public Command getStopCmd()
    {
        return runOnce(() ->
        {
            _flywheel.stop();
            _hood.stop();
        });
    }

    private final ShooterFlywheel _flywheel;
    private final ShooterHood     _hood;
    private final ShooterTurret   _turret;
    @Logged
    private ShooterState          _state     = ShooterState.Idle;
    @Logged
    private boolean               _hasTarget = false;

    public Shooter()
    {
        _flywheel = new ShooterFlywheel();
        _hood     = new ShooterHood();
        _turret   = new ShooterTurret();
    }

    @Override
    public void periodic()
    {
        _flywheel.periodic();
        _hood.periodic();
        _turret.periodic();

        if (_turret.hasTarget())
        {
            _turret.setState(ShooterTurret.TurretState.Track);
        }
        else
        {
            _turret.setState(ShooterTurret.TurretState.Off);
        }

        _hasTarget = _turret.hasTarget();

        updateState();
    }

    @Override
    public void simulationPeriodic()
    {
        _flywheel.simulationPeriodic();
        _hood.simulationPeriodic();
        _turret.simulationPeriodic();
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

    public double getDistanceToTarget()
    {
        return _turret.getDistanceToTarget();
    }

    public boolean atSetpoint()
    {
        return _turret.atSetpoint();
    }
}
