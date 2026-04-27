package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.TurretDirector.ShotMode;
import frc.robot.subsystems.shooter.TurretDirector.ShotSolution;

@Logged
public class Shooter extends SubsystemBase
{
    public enum ShooterState
    {
        Idle, Preparing, Ready, Firing, Manual, TrackingOnly;
    }

    public final Flywheel                    _flywheel;
    public final Feeder                      _feeder;
    public final Rotor                       _rotor;
    public final Turret                      _turret;
    public final TurretDirector              _turretDirector;
    private final Supplier<SwerveDriveState> _swerveStateSupplier;
    private final Supplier<Boolean>          _intakeExtendedSupplier;
    private final Supplier<Boolean>          _intakeRetractedSupplier;
    private final Debouncer                  _movingFeedDebouncer;
    @Logged
    private ShooterState                     _state;
    @Logged
    private boolean                          _autoShootEnabled;
    @Logged
    private ShotMode                         _requestedShotMode;
    private ShotSolution                     _currentShotSolution;
    @Logged
    private boolean                          _solutionReady;
    @Logged
    private boolean                          _turretReady;
    @Logged
    private boolean                          _flywheelReady;
    @Logged
    private boolean                          _hubReady;
    @Logged
    private boolean                          _inRangeReady;
    @Logged
    private boolean                          _rolloutReady;
    @Logged
    private boolean                          _stabilityReady;
    @Logged
    private boolean                          _usingMovingShotMath;
    @Logged
    private double                           _robotTranslationSpeedMetersPerSecond;
    @Logged
    private double                           _robotAngularSpeedRadiansPerSecond;
    @Logged
    private boolean                          _feedReady;

    public Shooter(Supplier<SwerveDriveState> swerveStateSupplier, Supplier<Boolean> intakeExtendedSupplier, Supplier<Boolean> intakeRetractedSupplier)
    {
        _swerveStateSupplier                  = swerveStateSupplier;
        _intakeExtendedSupplier               = intakeExtendedSupplier;
        _intakeRetractedSupplier              = intakeRetractedSupplier;
        _flywheel                             = new Flywheel();
        _feeder                               = new Feeder();
        _rotor                                = new Rotor();
        _turret                               = new Turret();
        _turretDirector                       = new TurretDirector(swerveStateSupplier);
        _movingFeedDebouncer                  = new Debouncer(ShooterConstants.SWM_FEED_STABILITY_WINDOW.in(Seconds));
        _state                                = ShooterState.Idle;
        _autoShootEnabled                     = false;
        _requestedShotMode                    = ShotMode.Idle;
        _currentShotSolution                  = createIdleSolution();
        _solutionReady                        = false;
        _turretReady                          = false;
        _flywheelReady                        = false;
        _hubReady                             = false;
        _inRangeReady                         = false;
        _rolloutReady                         = false;
        _stabilityReady                       = false;
        _usingMovingShotMath                  = false;
        _robotTranslationSpeedMetersPerSecond = 0.0;
        _robotAngularSpeedRadiansPerSecond    = 0.0;
        _feedReady                            = false;

        _feeder.set(false);
        _rotor.stop();
        _flywheel.stop();
        _turret.clearTargetAngle();
        _turret.setDisabled(false);
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

    public void setManualFlywheel(double rpm)
    {
        beginManualControl(false);
        _flywheel.setVelocity(RPM.of(Math.max(0.0, rpm)));
    }

    public void stopManualFlywheel()
    {
        _flywheel.stop();

        if (_state == ShooterState.Manual)
        {
            _state = ShooterState.Idle;
        }
    }

    public Command setFlywheelVelocity(AngularVelocity velocity)
    {
        return runOnce(() ->
        {
            beginManualControl(false);
            _flywheel.setVelocity(velocity);
        });
    }

    public Command runManualFeeder()
    {
        return startEnd(() ->
        {
            beginManualControl(false);
            _feeder.set(true);
            setRotorEnabled(true);
        }, () ->
        {
            _feeder.set(false);
            setRotorEnabled(false);
        });
    }

    public Command runFeeder()
    {
        return startEnd(() ->
        {
            _state = ShooterState.Manual;
            _feeder.set(true);
            setRotorEnabled(true);
        }, () ->
        {
            _feeder.set(false);
            setRotorEnabled(false);
            _flywheel.stop();
            _state = ShooterState.Idle;
        });
    }

    public Command stop()
    {
        return runOnce(this::stopShooter);
    }

    public Command homeTurret()
    {
        return setManualTurretAngle(ShooterConstants.TURRET_HOME_ANGLE);
    }

    public Command setManualTurretAngle(Angle angle)
    {
        return runOnce(() -> setManualTurretAngleCommand(angle));
    }

    public Command trackOnly()
    {
        return startEnd(() ->
        {
            _state             = ShooterState.TrackingOnly;
            _requestedShotMode = ShotMode.Track;
            _turret.setDisabled(false);
        }, this::stopShooter);
    }

    public void bumpManualTurretAngle(double deltaDeg)
    {
        setManualTurretAngle(_turret.getManualAngle().plus(Degrees.of(deltaDeg)));
    }

    @Override
    public void periodic()
    {
        switch (_state)
        {
            case Preparing:
            case Ready:
                _feeder.set(false);
                setRotorEnabled(false);
                runRequestedShot(true);

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
                runRequestedShot(true);
                if (isReadyToFeed())
                {
                    _feeder.set(true);
                    setRotorEnabled(true);
                }
                else
                {
                    _feeder.set(false);
                    setRotorEnabled(false);
                    _state = ShooterState.Preparing;
                }
                break;

            case Manual:
                if (_requestedShotMode != ShotMode.Idle)
                {
                    runRequestedShot(false);
                }
                break;

            case TrackingOnly:
                _feeder.set(false);
                setRotorEnabled(false);
                _flywheel.stop();
                _requestedShotMode = ShotMode.Track;
                runRequestedShot(false);
                break;

            case Idle:
            default:
                _flywheel.stop();
                _feeder.set(false);
                setRotorEnabled(false);
                clearShotRequest();
                break;
        }

        _turret.periodic();
        _flywheel.periodic();
        _feeder.periodic();
        _rotor.periodic();
    }

    @Override
    public void simulationPeriodic()
    {
        _flywheel.simulationPeriodic();
    }

    public void startPreparedShoot(boolean autoShoot)
    {
        beginShootingSequence(ShotMode.Track, autoShoot);
    }

    public void startPass(boolean autoShoot)
    {
        beginShootingSequence(ShotMode.Pass, autoShoot);
    }

    private void beginShootingSequence(ShotMode shotMode, boolean autoShoot)
    {
        _feeder.set(false);
        _turret.setDisabled(false);
        _requestedShotMode = shotMode;
        _autoShootEnabled  = autoShoot;
        _state             = ShooterState.Preparing;
    }

    private void beginManualControl(boolean clearTurretTarget)
    {
        _state               = ShooterState.Manual;
        _autoShootEnabled    = false;
        _requestedShotMode   = ShotMode.Idle;
        _currentShotSolution = createIdleSolution();
        clearReadinessGate();
        _feeder.set(false);
        setRotorEnabled(false);
        _turret.setDisabled(false);

        if (clearTurretTarget)
        {
            _turret.clearTargetAngle();
        }
    }

    private void setManualTurretAngleCommand(Angle angle)
    {
        beginManualControl(true);
        _turret.setManualAngle(angle);
    }

    public void stopShooter()
    {
        _state            = ShooterState.Idle;
        _autoShootEnabled = false;
        _turret.setDisabled(false);
        clearShotRequest();
    }

    public void commenceFiring()
    {
        if (_state == ShooterState.Ready)
        {
            _state = ShooterState.Firing;
        }
    }

    private void runRequestedShot(boolean spinFlywheel)
    {
        if (_requestedShotMode == ShotMode.Idle)
        {
            _turret.clearTargetAngle();
            _currentShotSolution = createIdleSolution();
            clearReadinessGate();
            return;
        }

        updateRobotMotionState();
        _usingMovingShotMath = shouldUseMovingShotMath();
        _currentShotSolution = _turretDirector.calculate(_requestedShotMode, _usingMovingShotMath);
        _turret.setTargetAngle(_currentShotSolution.turretAngle());

        if (spinFlywheel)
        {
            _flywheel.setVelocity(ShooterConstants.getFlywheelSpeedForDistance(_currentShotSolution.distance()));
        }
    }

    private void clearShotRequest()
    {
        _requestedShotMode   = ShotMode.Idle;
        _currentShotSolution = createIdleSolution();
        clearReadinessGate();
        setRotorEnabled(false);
    }

    private ShotSolution createIdleSolution()
    {
        return new ShotSolution(false, ShooterConstants.TURRET_HOME_ANGLE, Meters.zero(), ShotMode.Idle, new Pose2d(), false, false, false);
    }

    private boolean isReadyToFeed()
    {
        _solutionReady = _currentShotSolution.valid();
        _turretReady   = isTurretReadyForFeed();
        _flywheelReady = _flywheel.atSpeed();
        _hubReady      = hasActiveHubForCurrentShot();
        _inRangeReady  = hasValidDistanceForCurrentShot();
        _rolloutReady  = isWithinMovingFeedRolloutEnvelope();

        var rawFeedReady = _solutionReady && _turretReady && _flywheelReady && _inRangeReady && _rolloutReady;

        _stabilityReady = !_usingMovingShotMath || _movingFeedDebouncer.calculate(rawFeedReady);
        _feedReady      = rawFeedReady && _stabilityReady;
        return _feedReady;
    }

    private boolean isTurretReadyForFeed()
    {
        if (!_usingMovingShotMath || _currentShotSolution.mode() != ShotMode.Track)
        {
            return _turret.isLinedUp();
        }

        var movingToleranceDeg = ShooterConstants.SWM_FEED_TURRET_TOLERANCE.in(Degrees);
        var turretErrorDeg     = Math.abs(_turret.getTargetAngleError().in(Degrees));
        return turretErrorDeg <= movingToleranceDeg;
    }

    private boolean hasActiveHubForCurrentShot()
    {
        return switch (_currentShotSolution.mode())
        {
            case Track -> _currentShotSolution.hubActive();
            case Pass -> true;
            case Idle -> false;
        };
    }

    private boolean hasValidDistanceForCurrentShot()
    {
        return switch (_currentShotSolution.mode())
        {
            case Track -> !_usingMovingShotMath || _currentShotSolution.inRange();
            case Pass -> true;
            case Idle -> false;
        };
    }

    private boolean shouldUseMovingShotMath()
    {
        return _requestedShotMode == ShotMode.Track;
    }

    private boolean isWithinMovingFeedRolloutEnvelope()
    {
        if (!_usingMovingShotMath || _currentShotSolution.mode() != ShotMode.Track)
        {
            return true;
        }

        return _robotTranslationSpeedMetersPerSecond <= ShooterConstants.SWM_FEED_MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond)
                && Math.abs(_robotAngularSpeedRadiansPerSecond) <= ShooterConstants.SWM_FEED_MAX_ANGULAR_SPEED.in(RadiansPerSecond);
    }

    private void updateRobotMotionState()
    {
        var speeds = _swerveStateSupplier.get().Speeds;

        _robotTranslationSpeedMetersPerSecond = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        _robotAngularSpeedRadiansPerSecond    = speeds.omegaRadiansPerSecond;
    }

    private void clearReadinessGate()
    {
        _solutionReady                        = false;
        _turretReady                          = false;
        _flywheelReady                        = false;
        _hubReady                             = false;
        _inRangeReady                         = false;
        _rolloutReady                         = false;
        _stabilityReady                       = false;
        _usingMovingShotMath                  = false;
        _robotTranslationSpeedMetersPerSecond = 0.0;
        _robotAngularSpeedRadiansPerSecond    = 0.0;
        _movingFeedDebouncer.calculate(false);
        _feedReady = false;
    }

    private void setRotorEnabled(boolean enabled)
    {
        if (!enabled)
        {
            _rotor.stop();
            return;
        }

        if (_intakeExtendedSupplier.get())
        {
            _rotor.setVoltage(ShooterConstants.ROTOR_FAST_VOLTAGE);
            return;
        }

        var rotorVoltage = _intakeRetractedSupplier.get() ? ShooterConstants.ROTOR_RETRACTED_VOLTAGE : ShooterConstants.ROTOR_MID_VOLTAGE;
        _rotor.setVoltage(rotorVoltage);
    }
}
