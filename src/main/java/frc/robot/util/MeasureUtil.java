package frc.robot.util;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
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

    public static Angle clamp(final Angle value, final Angle min, final Angle max)
    {
        var unit = value.unit();
        return unit.of(clamp(unit, value, min, max));
    }

    public static Voltage clamp(final Voltage value, final Voltage min, final Voltage max)
    {
        var unit = value.unit();
        return unit.of(clamp(unit, value, min, max));
    }

    private static <U extends Unit> double clamp(final U unit, Measure<U> value, final Measure<U> min, final Measure<U> max)
    {
        return MathUtil.clamp(value.in(unit), min.in(unit), max.in(unit));
    }

    public static LinearVelocity applyDeadband(final LinearVelocity input, final Dimensionless deadband)
    {
        var unit = input.unit();
        return unit.of(MathUtil.applyDeadband(input.in(unit), deadband.in(Value)));
    }

    public static AngularVelocity applyDeadband(final AngularVelocity input, final Dimensionless deadband)
    {
        var unit = input.unit();
        return unit.of(MathUtil.applyDeadband(input.in(unit), deadband.in(Value)));
    }
}
