package frc.robot.subsystems.shooter;

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
        Idle, Preparing, Ready, Firing, Manual
    }

    /************
     * COMMANDS *
     ************/
    public Command startShooter()
    {
        return runOnce(() -> startShoot(false));
    }

    public Command fire()
    {
        return runOnce(this::commenceFiring);
    }

    public Command shoot()
    {
        return startEnd(() -> startShoot(true), () -> stopShooter());
    }

    public Command startPass()
    {
        return runOnce(() -> startPass(false));
    }

    public Command pass()
    {
        return startEnd(() -> startPass(true), () -> stopShooter());
    }

    public Command stop()
    {
        return runOnce(this::stopShooter);
    }

    // Manual commands below
    public Command setManualMode(boolean manual)
    {
        return runOnce(() ->
        {
            _state = manual ? ShooterState.Manual : ShooterState.Idle;
            _turret.setDisabled(manual);
        });
    }

    public Command setFlywheelVelocity(AngularVelocity velocity)
    {
        return runOnce(() -> _flywheel.setVelocity(velocity)).onlyIf(this::inManualMode);
    }

    public Command modFlywheelVelocity(AngularVelocity mod)
    {
        return runOnce(() -> _flywheel.setVelocity(_flywheel.getTargetVelocity().plus(mod))).onlyIf(this::inManualMode);
    }

    public Command stopFlywheel()
    {
        return runOnce(() -> _flywheel.stop()).onlyIf(this::inManualMode);
    }

    public Command runFeeder()
    {
        return startEnd(() -> _feeder.set(true), () -> _feeder.set(false)).onlyIf(this::inManualMode);
    }

    /*************
     * SUBSYSTEM *
     *************/
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

                if (_flywheel.atSpeed() && _turret.isLinedUp())
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
                _feeder.set(true);
                primeShot();
                break;

            case Manual:
                // Do nothing. The different parts of the
                // shooter will be controlled directly through
                // commands
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

    public void startShoot(boolean autoShoot)
    {
        beginShootingSequence(TurretState.Track, autoShoot);
    }

    public void startPass(boolean autoShoot)
    {
        beginShootingSequence(TurretState.Pass, autoShoot);
    }

    private void beginShootingSequence(TurretState turretState, boolean autoShoot)
    {
        if (_state == ShooterState.Idle)
        {
            _turret.setTurretState(turretState);
            _autoShootEnabled = autoShoot;
            _state            = ShooterState.Preparing;
        }
    }

    public void stopShooter()
    {
        _state            = ShooterState.Idle;
        _autoShootEnabled = false;
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

    private boolean inManualMode()
    {
        return _state == ShooterState.Manual;
    }
}
