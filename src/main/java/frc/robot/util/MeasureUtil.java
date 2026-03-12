package frc.robot.util;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public final class MeasureUtil
{
    // private constructor to prevent instantiation
    private MeasureUtil()
    {
    }

    public static Angle clamp(Angle value, Angle min, Angle max)
    {
        AngleUnit unit = value.unit();
        return unit.of(clamp(unit, value, min, max));
    }

    public static Voltage clamp(Voltage value, Voltage min, Voltage max)
    {
        VoltageUnit unit = value.unit();
        return unit.of(clamp(unit, value, min, max));
    }

    private static <U extends Unit> double clamp(U unit, Measure<U> value, Measure<U> min, Measure<U> max)
    {
        return MathUtil.clamp(value.in(unit), min.in(unit), max.in(unit));
    }

    public static LinearVelocity applyDeadband(LinearVelocity input, Dimensionless deadband)
    {
        LinearVelocityUnit unit = input.unit();
        return unit.of(MathUtil.applyDeadband(input.in(unit), deadband.in(Value)));
    }

    public static AngularVelocity applyDeadband(AngularVelocity input, Dimensionless deadband)
    {
        AngularVelocityUnit unit = input.unit();
        return unit.of(MathUtil.applyDeadband(input.in(unit), deadband.in(Value)));
    }
}
