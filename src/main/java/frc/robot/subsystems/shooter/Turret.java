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
    private final AnalogPotentiometer _turretSensor;
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
    private boolean                   _hasSetpoint;
    @Logged
    private Voltage                   _motorVoltage;
    @Logged
    private boolean                   _disabled;
    @Logged
    private boolean                   _linedUp;

    public Turret()
    {
        var sensorRange  = ShooterConstants.TURRET_HARD_MAX_ANGLE.minus(ShooterConstants.TURRET_HARD_MIN_ANGLE);
        var sensorOffset = ShooterConstants.TURRET_HARD_MIN_ANGLE;

        if (ShooterConstants.TURRET_SENSOR_INVERTED)
        {
            sensorRange  = sensorRange.unaryMinus();
            sensorOffset = sensorOffset.unaryMinus();
        }

        _turretMotor         = new TalonFX(CANConstants.TURRET_MOTOR);
        _turretSensor        = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER, sensorRange.in(Degrees), sensorOffset.in(Degrees));
        _pidController       = new PIDController(ShooterConstants.TURRET_KP, ShooterConstants.TURRET_KI, ShooterConstants.TURRET_KD);
        _controlMode         = ControlMode.Idle;
        _manualAngleSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
        _targetAngleSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
        _turretAngle         = Degrees.zero();
        _turretSetpoint      = ShooterConstants.TURRET_HOME_ANGLE;
        _hasSetpoint         = false;
        _motorVoltage        = Volts.zero();
        _disabled            = false;
        _linedUp             = false;

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT.in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.Clockwise_Positive;

        _turretMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig));

        _pidController.setTolerance(ShooterConstants.TURRET_TOLERANCE.in(Degrees));
        _pidController.setSetpoint(ShooterConstants.TURRET_HOME_ANGLE.in(Degrees));
    }

    public void periodic()
    {
        _rawTurretAngle = Degrees.of(_turretSensor.get());
        _turretAngle    = _rawTurretAngle.plus(ShooterConstants.TURRET_SENSOR_ZERO_OFFSET);
        _motorVoltage   = _turretMotor.getMotorVoltage().getValue();

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

        updateLinedUpState();

        if (_hasSetpoint && !_disabled)
        {
            motorOutput = Volts.of(_pidController.calculate(_turretAngle.in(Degrees), _turretSetpoint.in(Degrees)));

            if (_pidController.atSetpoint())
            {
                motorOutput = Volts.zero();
            }
        }

        motorOutput = applySoftLimit(motorOutput);
        _turretMotor.setVoltage(motorOutput.in(Volts));
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
        var currentDegrees   = currentAngle.in(Degrees);
        var requestedDegrees = requestedAngle.in(Degrees);
        var minDegrees       = ShooterConstants.TURRET_SOFT_MIN_ANGLE.in(Degrees);
        var maxDegrees       = ShooterConstants.TURRET_SOFT_MAX_ANGLE.in(Degrees);
        var wrapCenter       = (int)Math.round((currentDegrees - requestedDegrees) / 360.0);
        var bestAngle        = MathUtil.clamp(requestedDegrees, minDegrees, maxDegrees);
        var bestTravel       = Double.POSITIVE_INFINITY;
        var bestClampAmount  = Double.POSITIVE_INFINITY;
        var bestWrapDistance = Integer.MAX_VALUE;

        for (int wrapOffset = -kCandidateWrapCount; wrapOffset <= kCandidateWrapCount; wrapOffset++)
        {
            var wrapIndex      = wrapCenter + wrapOffset;
            var wrappedDegrees = requestedDegrees + 360.0 * wrapIndex;
            var legalDegrees   = MathUtil.clamp(wrappedDegrees, minDegrees, maxDegrees);
            var travelDegrees  = Math.abs(legalDegrees - currentDegrees);
            var clampAmount    = Math.abs(legalDegrees - wrappedDegrees);
            var wrapDistance   = Math.abs(wrapIndex);

            // Inspired by Team 5000's TurretCalculator and 6328's unwrap-style helpers:
            // choose a legal branch relative to the current azimuth instead of moduloing
            // first, so requests near +/- soft limits stay on the local side.
            var isBetterCandidate = travelDegrees < bestTravel || MathUtil.isNear(travelDegrees, bestTravel, kComparisonToleranceDeg)
                    && (clampAmount < bestClampAmount || MathUtil.isNear(clampAmount, bestClampAmount, kComparisonToleranceDeg) && (wrapDistance < bestWrapDistance || wrapDistance == bestWrapDistance && legalDegrees > bestAngle));

            if (isBetterCandidate)
            {
                bestAngle        = legalDegrees;
                bestTravel       = travelDegrees;
                bestClampAmount  = clampAmount;
                bestWrapDistance = wrapDistance;
            }
        }

        return Degrees.of(bestAngle);
    }

    private Voltage applySoftLimit(Voltage requestedVoltage)
    {
        if (_turretAngle.lte(ShooterConstants.TURRET_SOFT_MIN_ANGLE) && requestedVoltage.lt(Volts.zero()))
        {
            return Volts.zero();
        }

        if (_turretAngle.gte(ShooterConstants.TURRET_SOFT_MAX_ANGLE) && requestedVoltage.gt(Volts.zero()))
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
