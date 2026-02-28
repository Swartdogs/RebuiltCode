package frc.robot.subsystems.dashboard;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Hood.HoodPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.util.Utilities;

public class Dashboard extends SubsystemBase
{
    private final Intake           _intake;
    private final Shooter          _shooter;
    private final Turret           _turret;
    private final DoublePublisher  _matchTimePublisher;
    private final BooleanPublisher _hubActivePublisher;
    private final DoublePublisher  _hubStateTimeRemainingPublisher;
    private final BooleanPublisher _turretHasTargetPublisher;
    private final BooleanPublisher _turretLinedUpPublisher;
    private final BooleanPublisher _shooterShootModePublisher;
    private final StringPublisher  _shooterModePublisher;

    public Dashboard(Intake intake, Shooter shooter, Turret turret)
    {
        _intake  = intake;
        _shooter = shooter;
        _turret  = turret;

        var dashboardTable = NetworkTableInstance.getDefault().getTable("Dashboard");
        var valuesTable    = dashboardTable.getSubTable("Robot Values");

        _matchTimePublisher             = valuesTable.getDoubleTopic("Match Time").publish();
        _hubActivePublisher             = valuesTable.getBooleanTopic("Hub Active").publish();
        _hubStateTimeRemainingPublisher = valuesTable.getDoubleTopic("Hub State Time Remaining").publish();
        _turretHasTargetPublisher       = valuesTable.getBooleanTopic("Turret Has Target").publish();
        _turretLinedUpPublisher         = valuesTable.getBooleanTopic("Hub in Turret Range").publish();
        _shooterShootModePublisher      = valuesTable.getBooleanTopic("Shooter Is Shoot Mode").publish();
        _shooterModePublisher           = valuesTable.getStringTopic("Shooter Mode").publish();
    }

    @Override
    public void periodic()
    {
        _matchTimePublisher.set(Math.max(DriverStation.getMatchTime(), 0.0));

        boolean hubActive = Utilities.isHubActive();
        _hubActivePublisher.set(hubActive);
        _hubStateTimeRemainingPublisher.set(Math.max(Utilities.getHubStateTimeRemaining().in(Seconds), 0.0));

        if (_turret != null)
        {
            _turretHasTargetPublisher.set(_turret.hasTarget());
            _turretLinedUpPublisher.set(_turret.isLinedUp());
        }
        else
        {
            _turretHasTargetPublisher.set(false);
            _turretLinedUpPublisher.set(false);
        }

        HoodPosition shooterMode = _shooter.getShooterMode();
        _shooterShootModePublisher.set(shooterMode != HoodPosition.Pass);
        _shooterModePublisher.set(shooterMode.name());
    }
}
