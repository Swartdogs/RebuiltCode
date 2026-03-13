package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Turret.TurretState;

@Logged
public class Shooter extends SubsystemBase
{
    public enum ShooterState
    {
        Idle, Preparing, Ready, Firing, Manual;
    }

    public Command startShooter()
    {
        return runOnce(() -> startPreparedShoot(false));
    }

    public Command fire()
    {
        return runOnce(this::commenceFiring);
    }

    public Command shoot()
    {
        return startEnd(() -> startPreparedShoot(true), this::stopShooter);
    }

    public Command startPass()
    {
        return runOnce(() -> startPass(false));
    }

    public Command pass()
    {
        return startEnd(() -> startPass(true), this::stopShooter);
    }

    public Command setManualMode(boolean manual)
    {
        return runOnce(() ->
        {
            _state            = manual ? ShooterState.Manual : ShooterState.Idle;
            _autoShootEnabled = false;
            _feeder.set(false);
            _turret.setTurretState(TurretState.Idle);
            _turret.setDisabled(manual);

            if (manual)
            {
                _flywheel.stop();
            }
        });
    }

    public boolean inManualMode()
    {
        return _state == ShooterState.Manual;
    }

    public void setManualFlywheel(double rpm)
    {
        if (inManualMode())
        {
            _flywheel.setVelocity(RPM.of(Math.max(0.0, rpm)));
        }
    }

    public void stopManualFlywheel()
    {
        if (inManualMode())
        {
            _flywheel.stop();
        }
    }

    public Command setFlywheelVelocity(AngularVelocity velocity)
    {
        return runOnce(() ->
        {
            _state = ShooterState.Manual;
            _flywheel.setVelocity(velocity);
        });
    }

    public Command runManualFeeder()
    {
        return startEnd(() -> _feeder.set(true), () -> _feeder.set(false)).onlyIf(this::inManualMode);
    }

    public Command runFeeder()
    {
        return startEnd(() ->
        {
            _state = ShooterState.Manual;
            _feeder.set(true);
        }, () ->
        {
            _feeder.set(false);
            _flywheel.stop();
            _state = ShooterState.Idle;
        });
    }

    public Command stop()
    {
        return runOnce(this::stopShooter);
    }

    public final Flywheel _flywheel;
    public final Feeder   _feeder;
    public final Turret   _turret;
    @Logged
    private ShooterState  _state;
    @Logged
    private boolean       _autoShootEnabled;

    public Shooter(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        _flywheel         = new Flywheel();
        _feeder           = new Feeder();
        _turret           = new Turret(swerveStateSupplier);
        _state            = ShooterState.Idle;
        _autoShootEnabled = false;

        _feeder.set(false);
        _flywheel.stop();
        _turret.setTurretState(TurretState.Idle);
        _turret.setDisabled(false);
    }

    @Override
    public void periodic()
    {
        switch (_state)
        {
            case Preparing:
            case Ready:
                _feeder.set(false);
                primeShot();

                if (isReadyToFeed())
                {
                    if (_autoShootEnabled)
                    {
                        _state = ShooterState.Firing;
                    }
                    else
                    {
                        _state = ShooterState.Ready;
                    }
                }
                else
                {
                    _state = ShooterState.Preparing;
                }
                break;

            case Firing:
                primeShot();
                if (isReadyToFeed())
                {
                    _feeder.set(true);
                }
                else
                {
                    _feeder.set(false);
                    _state = ShooterState.Preparing;
                }
                break;

            case Manual:
                break;

            case Idle:
            default:
                _flywheel.stop();
                _feeder.set(false);
                _turret.setTurretState(TurretState.Idle);
                break;
        }

        _turret.periodic();
        _flywheel.periodic();
        _feeder.periodic();
    }

    @Override
    public void simulationPeriodic()
    {
        _flywheel.simulationPeriodic();
    }

    public void startPreparedShoot(boolean autoShoot)
    {
        beginShootingSequence(TurretState.Track, autoShoot);
    }

    public void startPass(boolean autoShoot)
    {
        beginShootingSequence(TurretState.Pass, autoShoot);
    }

    private void beginShootingSequence(TurretState turretState, boolean autoShoot)
    {
        _feeder.set(false);
        _turret.setDisabled(false);
        _turret.setTurretState(turretState);
        _autoShootEnabled = autoShoot;
        _state            = ShooterState.Preparing;
    }

    public void stopShooter()
    {
        _state            = ShooterState.Idle;
        _autoShootEnabled = false;
        _turret.setDisabled(false);
    }

    public void commenceFiring()
    {
        if (_state == ShooterState.Ready)
        {
            _state = ShooterState.Firing;
        }
    }

    private void primeShot()
    {
        if (_state != ShooterState.Idle)
        {
            var distance = _turret.getTargetDistance();
            _flywheel.setVelocity(ShooterConstants.getFlywheelSpeedForDistance(distance));
        }
    }

    private boolean isReadyToFeed()
    {
        return _flywheel.atSpeed();
    }
}
