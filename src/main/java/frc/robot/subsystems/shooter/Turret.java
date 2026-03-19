package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Turret
{
    private static final int    kCandidateWrapCount     = 2;
    private static final double kComparisonToleranceDeg = 1e-9;

    enum ControlMode
    {
        Idle, TargetAngle, ManualAngle
    }

    private final TalonFX                _turretMotor;
    private final AnalogPotentiometer    _turretPotentiometer;
    private final MotionMagicExpoVoltage _motionMagicRequest;
    private final NeutralOut             _neutralRequest;
    private ControlMode                  _controlMode;
    private boolean                      _encoderSeeded;
    @Logged
    private Angle                        _manualAngleSetpoint;
    @Logged
    private Angle                        _targetAngleSetpoint;
    @Logged
    private Angle                        _turretAngle;
    @Logged
    private Angle                        _turretSetpoint;
    @Logged
    private Angle                        _commandedTargetAngle;
    @Logged
    private Angle                        _targetAngleError;
    @Logged
    private boolean                      _hasSetpoint;
    @Logged
    private Voltage                      _motorVoltage;
    @Logged
    private boolean                      _disabled;
    @Logged
    private boolean                      _linedUp;
    @Logged
    private double                       _motorPositionRotations;

    public Turret()
    {
        var sensorRange  = ShooterConstants.TURRET_HARD_MAX_ANGLE.minus(ShooterConstants.TURRET_HARD_MIN_ANGLE);
        var sensorOffset = ShooterConstants.TURRET_HARD_MIN_ANGLE;

        if (ShooterConstants.TURRET_SENSOR_INVERTED)
        {
            sensorRange  = sensorRange.unaryMinus();
            sensorOffset = sensorOffset.unaryMinus();
        }

        _turretMotor            = new TalonFX(CANConstants.TURRET_MOTOR);
        _turretPotentiometer    = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER, sensorRange.in(Degrees), sensorOffset.in(Degrees));
        _motionMagicRequest     = new MotionMagicExpoVoltage(0);
        _neutralRequest         = new NeutralOut();
        _controlMode            = ControlMode.Idle;
        _encoderSeeded          = false;
        _manualAngleSetpoint    = ShooterConstants.TURRET_HOME_ANGLE;
        _targetAngleSetpoint    = ShooterConstants.TURRET_HOME_ANGLE;
        _turretAngle            = Degrees.zero();
        _turretSetpoint         = ShooterConstants.TURRET_HOME_ANGLE;
        _commandedTargetAngle   = ShooterConstants.TURRET_HOME_ANGLE;
        _targetAngleError       = Degrees.zero();
        _hasSetpoint            = false;
        _motorVoltage           = Volts.zero();
        _disabled               = false;
        _linedUp                = false;
        _motorPositionRotations = 0.0;

        var talonConfig = new TalonFXConfiguration();

        // Current limits
        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT.in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;

        // Motor output
        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.Clockwise_Positive;

        // Feedback (gear ratio: motor rotations to mechanism rotations)
        var feedbackConfig = new FeedbackConfigs();
        feedbackConfig.SensorToMechanismRatio = ShooterConstants.TURRET_GEAR_RATIO.magnitude();

        // PID Slot 0 gains
        var slot0Config = new Slot0Configs();
        slot0Config.kP = ShooterConstants.TURRET_KP;
        slot0Config.kS = ShooterConstants.TURRET_KS;

        // Motion Magic Expo settings
        var motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = ShooterConstants.TURRET_MM_CRUISE_VELOCITY.in(RotationsPerSecond);
        motionMagicConfig.MotionMagicAcceleration   = ShooterConstants.TURRET_MM_ACCELERATION.in(RotationsPerSecondPerSecond);
        motionMagicConfig.MotionMagicExpo_kV        = ShooterConstants.TURRET_MM_EXPO_KV;
        motionMagicConfig.MotionMagicExpo_kA        = ShooterConstants.TURRET_MM_EXPO_KA;

        // Software limits (in mechanism rotations, i.e. turret rotations)
        var softLimitConfig = new SoftwareLimitSwitchConfigs();
        softLimitConfig.ForwardSoftLimitEnable    = true;
        softLimitConfig.ForwardSoftLimitThreshold = ShooterConstants.TURRET_SOFT_MAX_ANGLE.in(Rotations);
        softLimitConfig.ReverseSoftLimitEnable    = true;
        softLimitConfig.ReverseSoftLimitThreshold = ShooterConstants.TURRET_SOFT_MIN_ANGLE.in(Rotations);

        talonConfig.withCurrentLimits(currentConfig).withMotorOutput(outputConfig).withFeedback(feedbackConfig).withSlot0(slot0Config).withMotionMagic(motionMagicConfig).withSoftwareLimitSwitch(softLimitConfig);

        _turretMotor.getConfigurator().apply(talonConfig);
    }

    public void periodic()
    {
        // Seed motor encoder from potentiometer on first run
        if (!_encoderSeeded)
        {
            var potAngle = Degrees.of(_turretPotentiometer.get());
            _turretMotor.setPosition(potAngle.in(Rotations));
            _encoderSeeded = true;
        }

        _motorPositionRotations = _turretMotor.getPosition().getValue().in(Rotations);
        _turretAngle            = Degrees.of(_motorPositionRotations * 360.0);
        _motorVoltage           = _turretMotor.getMotorVoltage().getValue();

        switch (_controlMode)
        {
            case TargetAngle:
                _hasSetpoint = true;
                _turretSetpoint = selectLegalSetpoint(_targetAngleSetpoint);
                break;

            case ManualAngle:
                _hasSetpoint = true;
                _turretSetpoint = selectLegalSetpoint(_manualAngleSetpoint);
                break;

            case Idle:
            default:
                _hasSetpoint = false;
                _turretSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
                break;
        }

        updateLinedUpState();

        _commandedTargetAngle = _turretSetpoint;
        _targetAngleError     = _commandedTargetAngle.minus(_turretAngle);

        if (_hasSetpoint && !_disabled)
        {
            var setpointRotations = _turretSetpoint.in(Degrees) / 360.0;
            _turretMotor.setControl(_motionMagicRequest.withPosition(setpointRotations));
        }
        else
        {
            _turretMotor.setControl(_neutralRequest);
        }
    }

    public void simulationPeriodic()
    {
    }

    public void setTargetAngle(Angle angle)
    {
        var targetDeltaDegrees = Math.abs(MathUtil.inputModulus(angle.minus(_targetAngleSetpoint).in(Degrees), -180.0, 180.0));

        if (_controlMode != ControlMode.TargetAngle || targetDeltaDegrees >= ShooterConstants.TURRET_TARGET_SETPOINT_DEADBAND.in(Degrees))
        {
            _targetAngleSetpoint = angle;
        }

        _controlMode = ControlMode.TargetAngle;
    }

    public void clearTargetAngle()
    {
        _controlMode = ControlMode.Idle;
    }

    public void setManualAngle(Angle angle)
    {
        _manualAngleSetpoint = angle;
        _controlMode         = ControlMode.ManualAngle;
    }

    public void bumpManualAngle(Angle delta)
    {
        _manualAngleSetpoint = _manualAngleSetpoint.plus(delta);
        _controlMode         = ControlMode.ManualAngle;
    }

    public Angle getManualAngle()
    {
        return _manualAngleSetpoint;
    }

    public boolean isLinedUp()
    {
        return _linedUp;
    }

    public Angle getTargetAngleError()
    {
        return _targetAngleError;
    }

    public void setDisabled(boolean disabled)
    {
        _disabled = disabled;
    }

    private Angle selectLegalSetpoint(Angle requestedAngle)
    {
        return chooseNearestLegalAngle(_turretAngle, requestedAngle);
    }

    static Angle chooseNearestLegalAngle(Angle currentAngle, Angle requestedAngle)
    {
        var    currentDegrees   = currentAngle.in(Degrees);
        var    requestedDegrees = requestedAngle.in(Degrees);
        var    minDegrees       = ShooterConstants.TURRET_SOFT_MIN_ANGLE.in(Degrees);
        var    maxDegrees       = ShooterConstants.TURRET_SOFT_MAX_ANGLE.in(Degrees);
        var    wrapCenter       = (int)Math.round((currentDegrees - requestedDegrees) / 360.0);
        var    bestTravel       = Double.POSITIVE_INFINITY;
        var    bestWrapDistance = Integer.MAX_VALUE;
        Double bestLegalAngle   = null;

        for (int wrapOffset = -kCandidateWrapCount; wrapOffset <= kCandidateWrapCount; wrapOffset++)
        {
            var wrapIndex      = wrapCenter + wrapOffset;
            var wrappedDegrees = requestedDegrees + 360.0 * wrapIndex;
            if (wrappedDegrees < minDegrees || wrappedDegrees > maxDegrees)
            {
                continue;
            }

            var travelDegrees = Math.abs(wrappedDegrees - currentDegrees);
            var wrapDistance  = Math.abs(wrapIndex);

            var isBetterCandidate = travelDegrees < bestTravel
                    || MathUtil.isNear(travelDegrees, bestTravel, kComparisonToleranceDeg) && (wrapDistance < bestWrapDistance || wrapDistance == bestWrapDistance && (bestLegalAngle == null || wrappedDegrees > bestLegalAngle));

            if (isBetterCandidate)
            {
                bestLegalAngle   = wrappedDegrees;
                bestTravel       = travelDegrees;
                bestWrapDistance = wrapDistance;
            }
        }

        if (bestLegalAngle != null)
        {
            return Degrees.of(bestLegalAngle);
        }

        return Degrees.of(MathUtil.clamp(requestedDegrees, minDegrees, maxDegrees));
    }

    private void updateLinedUpState()
    {
        if (!_hasSetpoint || _disabled)
        {
            _linedUp = false;
            return;
        }

        var angleErrorDegrees   = Math.abs(_turretSetpoint.minus(_turretAngle).in(Degrees));
        var acquireTolerance    = ShooterConstants.TURRET_TOLERANCE.in(Degrees);
        var holdTolerance       = ShooterConstants.TURRET_LINED_UP_HOLD_TOLERANCE.in(Degrees);
        var allowedErrorDegrees = _linedUp ? holdTolerance : acquireTolerance;

        _linedUp = angleErrorDegrees <= allowedErrorDegrees;
    }
}
