package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

public class IntakeIOSim implements IntakeIO
{
    private double _appliedVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        inputs.motorVolts = _appliedVolts;
    }

    @Override
    public void setVolts(double volts)
    {
        _appliedVolts = MathUtil.clamp(volts, -12, 12);
    }
}