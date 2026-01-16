package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelIO
{
    @AutoLog
    public static class ShooterFlywheelIOInputs
    {
        public double   velocity = 0.0;
        public double   volts    = 0.0;
    }

    public default void updateInputs(ShooterFlywheelIOInputs inputs)
    {
    }

    public default void setVelocity(double velocity)
    {
    }
}