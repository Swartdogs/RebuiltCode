package frc.robot.subsystems.intake;
import edu.wpi.first.epilogue.Logged; 

public interface IntakeIO
{
    @Logged
    public static class IntakeIOInputs
    {
        public double   motorVolts   = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs)
    {

    }

    public default void setVolts(double volts)
    {

    }
}