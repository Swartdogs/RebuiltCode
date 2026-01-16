package frc.robot.subsystems.turret;

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
    public default double getVelocity()
    {
        return 0. ;
    }

    public default void stop()
    {

    }
}