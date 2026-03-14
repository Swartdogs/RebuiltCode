package frc.robot.subsystems.dashboard;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret.TurretState;
import frc.robot.util.Utilities;

public class Dashboard extends SubsystemBase
{
    // Assumed teleop length when FMS match time is unavailable
    private static final double TELEOP_SECS = 135.0;

    // Elapsed-time shift boundaries (derived from Utilities constants, assuming
    // 135s teleop)
    // kTransitionShiftEnd = 130s remaining → 5s elapsed
    // kShift1End = 105s remaining → 30s elapsed
    // kShift2End = 80s remaining → 55s elapsed
    // kShift3End = 55s remaining → 80s elapsed
    // kShift4End = 30s remaining → 105s elapsed
    private static final double[]  SHIFT_ELAPSED_ENDS = { 5.0, 30.0, 55.0, 80.0, 105.0 };
    private final Shooter          _shooter;
    private final DoublePublisher  _matchTimePub;
    private final BooleanPublisher _hubActivePub;
    private final DoublePublisher  _hubShiftTimerPub;
    private final BooleanPublisher _shooterIsShootModePub;
    private final BooleanPublisher _turretHasTargetPub;
    private final BooleanPublisher _turretLinedUpPub;
    private final Timer            _teleopTimer       = new Timer();
    private boolean                _wasInTeleop       = false;

    public Dashboard(Shooter shooter)
    {
        _shooter = shooter;

        var table = NetworkTableInstance.getDefault().getTable("Dashboard").getSubTable("Robot Values");

        _matchTimePub          = table.getDoubleTopic("Match Time").publish();
        _hubActivePub          = table.getBooleanTopic("Hub Active").publish();
        _hubShiftTimerPub      = table.getDoubleTopic("Hub State Time Remaining").publish();
        _shooterIsShootModePub = table.getBooleanTopic("Shooter Is Shoot Mode").publish();
        _turretHasTargetPub    = table.getBooleanTopic("Turret Has Target").publish();
        _turretLinedUpPub      = table.getBooleanTopic("Turret Lined Up").publish();
    }

    @Override
    public void periodic()
    {
        updateTeleopTimer();

        _matchTimePub.set(getMatchTime());
        _hubActivePub.set(Utilities.isHubActive());
        _hubShiftTimerPub.set(getShiftTimer());
        _shooterIsShootModePub.set(_shooter._turret.getTurretState() == TurretState.Track);
        _turretHasTargetPub.set(_shooter._turret.hasLimelightTarget());
        _turretLinedUpPub.set(_shooter._turret.isLinedUp());
    }

    private void updateTeleopTimer()
    {
        boolean inTeleop = DriverStation.isTeleop() && DriverStation.isEnabled();

        if (inTeleop && !_wasInTeleop)
        {
            _teleopTimer.restart();
        }
        else if (!DriverStation.isTeleop())
        {
            _teleopTimer.stop();
            _teleopTimer.reset();
        }

        _wasInTeleop = inTeleop;
    }

    private double getMatchTime()
    {
        double fmsTime = DriverStation.getMatchTime();
        if (fmsTime >= 0) return fmsTime;
        if (!DriverStation.isTeleop()) return -1;

        // Fallback: count down from assumed teleop length
        return Math.max(0, TELEOP_SECS - _teleopTimer.get());
    }

    private double getShiftTimer()
    {
        double fmsTime = DriverStation.getMatchTime();
        if (fmsTime >= 0) return Utilities.getTimeUntilNextShift();
        if (!DriverStation.isTeleop()) return -1;

        // Fallback: compute from local elapsed timer
        double elapsed = _teleopTimer.get();
        for (double end : SHIFT_ELAPSED_ENDS)
        {
            if (elapsed < end) return end - elapsed;
        }
        return Math.max(0, TELEOP_SECS - elapsed); // endgame
    }
}
