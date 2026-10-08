package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/*
 * Interface for objects that represent the physical layer (motors and sensors)
 * of the flywheel sub-component of the shooter subsystem.
 */
public interface FlywheelIO
{
    /*
     * A container for the sensor inputs we can expect back from the flywheel each
     * timestep.
     */
    @Logged
    public static class FlywheelIOInputs
    {
        // The voltage currently applied to the left flywheel motor
        @Logged
        public Voltage leftMotorAppliedVoltage = Volts.zero();

        // The amount of current the left flywheel motor is currently drawing
        @Logged
        public Current leftMotorCurrentDraw = Amps.zero();

        // The current rotational velocity of the left flywheel motor
        @Logged
        public AngularVelocity leftMotorRotationRate = RPM.zero();

        // The voltage currently applied to the right flywheel motor
        @Logged
        public Voltage rightMotorAppliedVoltage = Volts.zero();

        // The amound of current the right flywheel motor is currently drawing
        @Logged
        public Current rightMotorCurrentDraw = Amps.zero();

        // The current rotational velocity of the right flywheel motor
        @Logged
        public AngularVelocity rightMotorRotationRate = RPM.zero();
    }

    // Allows the FlywheelIO object to read sensor inputs. Updated values are loaded
    // into the provided "inputs" parameter.
    public void updateInputs(FlywheelIOInputs inputs);

    // Sets the desired angular rate of rotation of the flywheel
    public void setRate(AngularVelocity rate);

    // Commands the flywheel to stop rotating
    public void stop();
}
