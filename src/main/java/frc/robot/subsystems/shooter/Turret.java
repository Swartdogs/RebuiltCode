package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Turret extends SubsystemBase
{
    public enum TurretState
    {
        Idle, Shoot, Pass
    }

    public Turret(Supplier<SwerveDriveState> swerveStateSupplier)
    {
    }

    /**
     * This function executes automatically every 20 milliseconds.
     * For the turret specifically, we need to ensure the turret
     * is behaving according to the rules defined by the state the
     * turret is in.
     * 
     * If in <code>Idle</code>, the turret shouldn't rotate at all,
     * even if the robot rotates.
     * 
     * If in <code>Shoot</code> or <code>Pass</code>, the turret
     * should rotate to whatever angle is provided by the <code>
     * TurretDirector</code> class.
     */
    @Override
    public void periodic()
    {
    }

    /**
     * Any logic needed to specifically manage any hardware interactions
     * when the robot code is executing in the simulation (for example,
     * update the 10-turn potentiometer's angle based on how the motor
     * driving the turret is moving) should go here.
     */
    @Override
    public void simulationPeriodic()
    {
    }

    /**
     * Sets the desired state of the turret.
     * 
     * @param state The desired state of the turret
     */
    public void setTurretState(TurretState state)
    {
    }

    /**
     * Gets the current state of the turret.
     * 
     * @return The current state of the turret
     */
    public TurretState getTurretState()
    {
        return TurretState.Idle;
    }

    /**
     * Gets the current angle of the turret. Angle is given
     * in field-relative space.
     * 
     * @return The current angle of the turret
     */
    public Angle getAngle()
    {
        return Degrees.zero();
    }

    /**
     * Gets whether the turret is currently pointing at its
     * desired target. The target to point at is determined
     * by the state of the turret.
     * 
     * @return <code>true</code> if the turret is facing the desired
     *         target location (or if there is no desired
     *         location), otherwise <code>false</code>.
     */
    public boolean isLinedUp()
    {
        return false;
    }

    /**
     * The TurretDirector is responsible for choosing a target for
     * the robot to aim at and communicating the desired turret angle
     * in field-relative space back to the main Turret subsystem.
     */
    private static class TurretDirector
    {
    }
}
