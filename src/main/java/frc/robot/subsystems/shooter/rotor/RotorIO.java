package frc.robot.subsystems.shooter.rotor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/*
 * Interface for objects that represent the physical layer (motors and sensors)
 * of the rotor sub-component of the shooter subsystem.
 */
public interface RotorIO
{
    /*
     * A container for the sensor inputs we can expect back from the rotor motor
     * each timestep.
     */
    @Logged
    public static class RotorIOInputs
    {
        // The voltage the rotor is currently applying
        @Logged
        public Voltage appliedVoltage = Volts.zero();

        // The amount of current the rotor is currently drawing
        @Logged
        public Current currentDraw = Amps.zero();

        // The current rotational velocity of the rotor
        @Logged
        public AngularVelocity rotationRate = RPM.zero();
    }

    // Allows the RotorIO object to read sensor inputs. Updated values are
    // loaded into the provided "inputs" parameter.
    public void updateInputs(RotorIOInputs inputs);

    // Sets the desired angular rate of rotation of the rotor
    public void setRate(AngularVelocity rate);

    // Commands the rotor to stop rotating
    public void stop();
}
