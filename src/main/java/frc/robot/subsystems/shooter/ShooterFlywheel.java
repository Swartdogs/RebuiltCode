package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
    private final SparkFlex                 _leadMotor;
    private final SparkFlex                 _followMotor;
    private final SparkClosedLoopController _closedLoopControl;
    private final RelativeEncoder           _encoder;
    private double                          _velocity;
    private double                          _velocityTarget;
    private double                          _maxSpeed = Constants.Shooter.MAX_FLYWHEEL_SPEED * Constants.General.MAX_NEO_SPEED;

    public ShooterFlywheel()
    {
        if (RobotBase.isSimulation())
        {
            _leadMotor         = null;
            _followMotor       = null;
            _closedLoopControl = null;
            _encoder           = null;
            return;
        }
        _leadMotor   = new SparkFlex(Constants.CAN.FLYWHEEL_LEAD, MotorType.kBrushless);
        _followMotor = new SparkFlex(Constants.CAN.FLYWHEEL_FOLLOW, MotorType.kBrushless);
        _leadMotor.setCANTimeout(250);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(Constants.Shooter.FLYWHEEL_CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE);
        config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control.
                // We don't need to pass a closed loop slot, as it will default to slot 0.
                .p(0.1).d(0).outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(Constants.Shooter.FLYWHEEL_KP, ClosedLoopSlot.kSlot1).d(Constants.Shooter.FLYWHEEL_KD, ClosedLoopSlot.kSlot1).outputRange(-1, 1, ClosedLoopSlot.kSlot1).feedForward
                        // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);
        _leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(Constants.Shooter.FLYWHEEL_CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE).follow(_leadMotor, true);
        _followMotor.setCANTimeout(250);
        _followMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        _closedLoopControl = _leadMotor.getClosedLoopController();
        _encoder           = _leadMotor.getEncoder();
        _leadMotor.setCANTimeout(0);
        _followMotor.setCANTimeout(0);
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
        _closedLoopControl.setSetpoint(_velocityTarget, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
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
