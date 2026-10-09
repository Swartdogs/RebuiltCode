package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

/*
 * A real implementation of the physical layer (motors and sensors) of the
 * flywheel sub-component of the shooter subsystem.
 */
public class FlywheelIOReal implements FlywheelIO
{
    // The left flywheel motor
    private final SparkFlex _leftFlywheelMotor;

    // The right flywheel motor
    private final SparkFlex _rightFlywheelMotor;

    // The closed loop controller for running the flywheel at a specified velocity
    private final SparkClosedLoopController _pid;

    // The encoder built into the left flywheel motor
    private final RelativeEncoder _leftFlywheelEncoder;

    // The encoder built into the right flywheel motor
    private final RelativeEncoder _rightFlywheelEncoder;

    public FlywheelIOReal()
    {
        // Instantiate the motors
        _leftFlywheelMotor  = new SparkFlex(CANConstants.FLYWHEEL_LEAD, MotorType.kBrushless);
        _rightFlywheelMotor = new SparkFlex(CANConstants.FLYWHEEL_FOLLOW, MotorType.kBrushless);

        // Set the CAN timeout high for initial configuration
        _leftFlywheelMotor.setCANTimeout(250); // Milliseconds
        _rightFlywheelMotor.setCANTimeout(250); // Milliseconds

        // Start configuring the motors
        var config = new SparkFlexConfig();

        // Motor configurations related to general motor output
        config.inverted(false);
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit((int)ShooterConstants.FLYWHEEL_CURRENT_LIMIT.in(Amps));
        config.voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        // Motor configurations related to closed loop gains
        config.closedLoop.feedForward.kG(0.0);
        config.closedLoop.feedForward.kS(ShooterConstants.FLYWHEEL_KS.in(Volts));
        config.closedLoop.feedForward.kV(ShooterConstants.FLYWHEEL_KV.in(Volts.per(RPM)));
        config.closedLoop.feedForward.kA(ShooterConstants.FLYWHEEL_KA.in(Volts.per(RotationsPerSecondPerSecond)));
        config.closedLoop.p(ShooterConstants.FLYWHEEL_KP.in(Volts.per(RPM)));
        config.closedLoop.i(0.0);
        config.closedLoop.d(ShooterConstants.FLYWHEEL_KD.in(Volts.per(RPM.per(Second))));

        // Configure the leader motor
        _leftFlywheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Provide an additional configuration for the follower motor
        config.follow(_leftFlywheelMotor);

        // Configure the follower motor
        _rightFlywheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get the encoders
        _leftFlywheelEncoder  = _leftFlywheelMotor.getEncoder();
        _rightFlywheelEncoder = _leftFlywheelMotor.getEncoder();

        // The closed loop controller
        _pid = _leftFlywheelMotor.getClosedLoopController();

        // Reset the CAN timeouts
        _leftFlywheelMotor.setCANTimeout(0);
        _rightFlywheelMotor.setCANTimeout(0);
    }

    // Allows the FlywheelIOReal object to read sensor inputs. Updated values are
    // loaded into the provided "inputs" parameter.
    @Override
    public void updateInputs(FlywheelIOInputs inputs)
    {
        inputs.leftMotorAppliedVoltage = Volts.of(_leftFlywheelMotor.getBusVoltage()).times(_leftFlywheelMotor.getAppliedOutput());
        inputs.leftMotorCurrentDraw    = Amps.of(_leftFlywheelMotor.getOutputCurrent());
        inputs.leftMotorRotationRate   = RPM.of(_leftFlywheelEncoder.getVelocity());

        inputs.rightMotorAppliedVoltage = Volts.of(_rightFlywheelMotor.getBusVoltage()).times(_rightFlywheelMotor.getAppliedOutput());
        inputs.rightMotorCurrentDraw    = Amps.of(_rightFlywheelMotor.getOutputCurrent());
        inputs.rightMotorRotationRate   = RPM.of(_rightFlywheelEncoder.getVelocity());
    }

    // Sets the desired angular rate of rotation of the flywheel.
    @Override
    public void setRate(AngularVelocity rate)
    {
        _pid.setSetpoint(rate.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    // Commands the flywheel to stop rotating.
    @Override
    public void stop()
    {
        _leftFlywheelMotor.setVoltage(Volts.zero());
    }
}
