package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;

/*
 * Hardware The following motors and sensors will need to be added to the
 * subsystem for controlling the hardware Motors Flywheel (2x NEO Vortex motors
 * driven by SPARK Flex motor controllers) Hood (1x Motor of undetermined type)
 * Turret (1x Kraken x44 driven by a TalonFX motor controller) Sensors Hood
 * angle absolute encoder (1x sensor of undetermined type) Turret angle absolute
 * encoder (1x sensor of undetermined type) Camera (1x Limelight)
 */

public class ShooterFlywheel extends SubsystemBase
{
    private final SparkFlex                 _motor;
    private final SparkClosedLoopController _control;
    private final RelativeEncoder           _encoder;
    private double                          _velocity;
    private double                          _velocityTarget;
    private double                          _maxSpeed      = Constants.Shooter.MAX_FLYWHEEL_SPEED * Constants.General.MAX_NEO_SPEED;
    private double                          _velocityRange = Constants.Shooter.VELOCITY_RANGE;

    private SparkFlexConfig config()
    {
        SparkFlexConfig val = new SparkFlexConfig();

        val.encoder.positionConversionFactor(1).velocityConversionFactor(1);

        val.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control.
                // We don't need to pass a closed loop slot, as it will default to slot 0.
                .p(0.1).i(0).d(0).outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1).i(0, ClosedLoopSlot.kSlot1).d(0, ClosedLoopSlot.kSlot1).outputRange(-1, 1, ClosedLoopSlot.kSlot1).feedForward
                        // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

        return val;
    }

    private void initMotor(SparkFlex motor)
    {
        motor.setCANTimeout(250);

        // set PID config
        motor.configure(config(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        motor.setCANTimeout(0);
    }

    public ShooterFlywheel()
    {
        _motor   = new SparkFlex(Constants.CAN.SHOOTERFLYWHEEL, MotorType.kBrushless);
        _control = _motor.getClosedLoopController();
        _encoder = _motor.getEncoder();

        initMotor(_motor);
    }

    @Override
    public void periodic()
    {
        _velocity = _encoder.getVelocity();
        // updateInputs(_inputs);
    }

    public boolean atSpeed()
    {
        return (_velocityTarget > 0.) && (_velocity > _velocityTarget);
    }

    public void setVelocity(double velocity)
    {
        _velocityTarget = MathUtil.clamp(velocity, 0, _maxSpeed);
        // Logger.recordOutput("Shooter/Flywheel/UpperVelocitySetpoint",
        // _velocityTarget);
        _control.setSetpoint(_velocityTarget, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void stop()
    {
        setVelocity(0.);
    }

    public void setMaxSpeed(double maxFlywheelSpeed)
    {
        _maxSpeed = maxFlywheelSpeed * Constants.General.MAX_NEO_SPEED;
    }
}
