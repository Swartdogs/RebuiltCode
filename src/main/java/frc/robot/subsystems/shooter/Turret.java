package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

    private final TalonFX             _turretMotor;
    private final AnalogPotentiometer _turretPotentiometer;
    private final PIDController       _pidController;
    private ControlMode               _controlMode;
    @Logged
    private Angle                     _manualAngleSetpoint;
    @Logged
    private Angle                     _targetAngleSetpoint;
    @Logged
    private Angle                     _turretAngle;
    @Logged
    private Angle                     _rawTurretAngle;
    @Logged
    private Angle                     _turretSetpoint;
    @Logged
    private Angle                     _commandedTargetAngle;
    @Logged
    private Angle                     _targetAngleError;
    @Logged
    private boolean                   _hasSetpoint;
    @Logged
    private Voltage                   _motorVoltage;
    private Voltage                   _lastCommandedMotorVoltage;
    @Logged
    private boolean                   _disabled;
    @Logged
    private boolean                   _linedUp;
    @Logged
    private double                    _motorPositionRotations;

    public Turret()
    {
        var sensorRange  = ShooterConstants.TURRET_HARD_MAX_ANGLE.minus(ShooterConstants.TURRET_HARD_MIN_ANGLE);
        var sensorOffset = ShooterConstants.TURRET_HARD_MIN_ANGLE;

        if (ShooterConstants.TURRET_SENSOR_INVERTED)
        {
            sensorRange  = sensorRange.unaryMinus();
            sensorOffset = sensorOffset.unaryMinus();
        }

        _turretMotor               = new TalonFX(CANConstants.TURRET_MOTOR);
        _turretPotentiometer       = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER, sensorRange.in(Degrees), sensorOffset.in(Degrees));
        _pidController             = new PIDController(ShooterConstants.TURRET_KP, ShooterConstants.TURRET_KI, ShooterConstants.TURRET_KD);
        _controlMode               = ControlMode.Idle;
        _manualAngleSetpoint       = ShooterConstants.TURRET_HOME_ANGLE;
        _targetAngleSetpoint       = ShooterConstants.TURRET_HOME_ANGLE;
        _turretAngle               = Degrees.zero();
        _rawTurretAngle            = Degrees.zero();
        _turretSetpoint            = ShooterConstants.TURRET_HOME_ANGLE;
        _commandedTargetAngle      = ShooterConstants.TURRET_HOME_ANGLE;
        _targetAngleError          = Degrees.zero();
        _hasSetpoint               = false;
        _motorVoltage              = Volts.zero();
        _lastCommandedMotorVoltage = Volts.zero();
        _disabled                  = false;
        _linedUp                   = false;
        _motorPositionRotations    = 0.0;

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT.in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.Clockwise_Positive;

        _turretMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig));
        _pidController.setTolerance(ShooterConstants.TURRET_TOLERANCE.in(Degrees));
    }

    public void periodic()
    {
        _rawTurretAngle         = Degrees.of(_turretPotentiometer.get());
        _turretAngle            = _rawTurretAngle.plus(ShooterConstants.TURRET_POT_OFFSET);
        _motorPositionRotations = _turretMotor.getPosition().getValue().baseUnitMagnitude();
        _motorVoltage           = _turretMotor.getMotorVoltage().getValue();

        var motorOutput = Volts.zero();

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

        if (_hasSetpoint)
        {
            _turretSetpoint = limitSetpointStep(_commandedTargetAngle, _turretSetpoint);
        }

        updateLinedUpState();

        _commandedTargetAngle = _turretSetpoint;
        _targetAngleError     = _commandedTargetAngle.minus(_turretAngle);

        if (_hasSetpoint && !_disabled)
        {
            var errorDegrees  = _turretSetpoint.minus(_turretAngle).in(Degrees);
            var outputVolts   = _pidController.calculate(_turretAngle.in(Degrees), _turretSetpoint.in(Degrees));
            var staticFFVolts = ShooterConstants.TURRET_STATIC_FF.in(Volts);

            if (!MathUtil.isNear(0.0, errorDegrees, ShooterConstants.TURRET_STATIC_FF_ERROR_DEADBAND.in(Degrees)))
            {
                outputVolts += Math.copySign(staticFFVolts, errorDegrees);
            }

            motorOutput = Volts.of(outputVolts);
        }

        motorOutput = Volts.of(limitOutputStep(motorOutput.in(Volts)));
        motorOutput = applySoftLimit(motorOutput);
        _turretMotor.setVoltage(motorOutput.in(Volts));
        _lastCommandedMotorVoltage = motorOutput;
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

    private Angle limitSetpointStep(Angle previousSetpoint, Angle requestedSetpoint)
    {
        var maxStepDeg      = ShooterConstants.TURRET_MAX_SETPOINT_STEP_PER_LOOP.in(Degrees);
        var deltaDeg        = requestedSetpoint.minus(previousSetpoint).in(Degrees);
        var limitedDeltaDeg = MathUtil.clamp(deltaDeg, -maxStepDeg, maxStepDeg);
        var limitedSetpoint = previousSetpoint.plus(Degrees.of(limitedDeltaDeg));
        return selectLegalSetpoint(limitedSetpoint);
    }

    private double limitOutputStep(double requestedVolts)
    {
        var previousVolts     = _lastCommandedMotorVoltage.in(Volts);
        var maxStepVolts      = ShooterConstants.TURRET_MAX_OUTPUT_STEP_PER_LOOP.in(Volts);
        var deltaVolts        = requestedVolts - previousVolts;
        var limitedDeltaVolts = MathUtil.clamp(deltaVolts, -maxStepVolts, maxStepVolts);
        return previousVolts + limitedDeltaVolts;
    }

    private Voltage applySoftLimit(Voltage requestedVoltage)
    {
        var requestedVolts = requestedVoltage.in(Volts);
        var turretDegrees  = _turretAngle.in(Degrees);

        if (turretDegrees <= ShooterConstants.TURRET_SOFT_MIN_ANGLE.in(Degrees) && requestedVolts < 0.0)
        {
            return Volts.zero();
        }

        if (turretDegrees >= ShooterConstants.TURRET_SOFT_MAX_ANGLE.in(Degrees) && requestedVolts > 0.0)
        {
            return Volts.zero();
        }

        return requestedVoltage;
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
