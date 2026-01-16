package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO
{
    private double _appliedVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        inputs.motorVolts   = _appliedVolts;
    }

    @Override
    public void setVolts(double volts)
    {
        _appliedVolts = MathUtil.clamp(_appliedVolts, -12, 12);
    }
}