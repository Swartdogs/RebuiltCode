package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TurretAngleUtilTest
{
    private static final double kAngleToleranceDeg = 1e-9;

    @Test
    void keepsAlreadyLegalTarget()
    {
        assertNearestLegalAngle(12.0, 35.0, 35.0);
    }

    @Test
    void wrapsToPositiveEquivalentNearUpperSoftLimit()
    {
        assertNearestLegalAngle(150.0, -200.0, 160.0);
    }

    @Test
    void clampsTowardCurrentBranchWhenPositiveRequestIsPastLimit()
    {
        assertNearestLegalAngle(150.0, 190.0, 160.0);
    }

    @Test
    void clampsTowardCurrentBranchWhenNegativeRequestIsPastLimit()
    {
        assertNearestLegalAngle(-150.0, -190.0, -160.0);
    }

    @Test
    void breaksCenterTieDeterministically()
    {
        assertNearestLegalAngle(0.0, 180.0, 160.0);
    }

    private void assertNearestLegalAngle(double currentDegrees, double requestedDegrees, double expectedDegrees)
    {
        var actual = Turret.chooseNearestLegalAngle(Degrees.of(currentDegrees), Degrees.of(requestedDegrees));

        assertEquals(expectedDegrees, actual.in(Degrees), kAngleToleranceDeg);
    }
}
