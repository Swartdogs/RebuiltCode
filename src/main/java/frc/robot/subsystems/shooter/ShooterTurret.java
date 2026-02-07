package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class ShooterTurret
{
    public enum TurretState
    {
        Off, Track, Pass
    }

    private final Turret _turret;

    public ShooterTurret()
    {
        this(null);
    }

    public ShooterTurret(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        _turret = new Turret(swerveStateSupplier);
    }

    public void periodic()
    {
        _turret.periodic();
    }

    public void simulationPeriodic()
    {
        _turret.simulationPeriodic();
    }

    public void setState(TurretState state)
    {
        _turret.setTurretState(convertState(state));
    }

    public TurretState getState()
    {
        return convertState(_turret.getTurretState());
    }

    public boolean atSetpoint()
    {
        return _turret.isLinedUp();
    }

    public boolean hasTarget()
    {
        return _turret.hasTarget();
    }

    public double getDistanceToTarget()
    {
        return _turret.getDistanceToTarget();
    }

    private static Turret.TurretState convertState(TurretState state)
    {
        if (state == null) return Turret.TurretState.Idle;

        return switch (state)
        {
            case Off -> Turret.TurretState.Idle;
            case Track -> Turret.TurretState.Track;
            case Pass -> Turret.TurretState.Pass;
        };
    }

    private static TurretState convertState(Turret.TurretState state)
    {
        if (state == null) return TurretState.Off;

        return switch (state)
        {
            case Idle -> TurretState.Off;
            case Track -> TurretState.Track;
            case Pass -> TurretState.Pass;
        };
    }
}
