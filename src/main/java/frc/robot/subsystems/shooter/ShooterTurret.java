package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterTurret extends SubsystemBase
{
    public enum TurretState 
    {
        Off, Track, Pass 
    }

    private TurretState _state; 

    public ShooterTurret() 
    {
    }

    public TurretState getState() 
    {
        return _state; 
    }
}
