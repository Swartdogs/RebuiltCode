package frc.robot.subsystems.dashboard;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret.TurretState;
import frc.robot.util.Utilities;

public class Dashboard extends SubsystemBase
{
    private final Shooter          _shooter;
    private final DoublePublisher  _matchTimePub;
    private final BooleanPublisher _hubActivePub;
    private final DoublePublisher  _hubShiftTimerPub;
    private final BooleanPublisher _shooterIsShootModePub;
    private final BooleanPublisher _turretHasTargetPub;
    private final BooleanPublisher _turretLinedUpPub;

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
        _matchTimePub.set(DriverStation.getMatchTime());
        _hubActivePub.set(Utilities.isHubActive());
        _hubShiftTimerPub.set(Utilities.getTimeUntilNextShift());
        _shooterIsShootModePub.set(_shooter._turret.getTurretState() == TurretState.Track);
        _turretHasTargetPub.set(_shooter._turret.hasLimelightTarget());
        _turretLinedUpPub.set(_shooter._turret.isLinedUp());
    }
}
