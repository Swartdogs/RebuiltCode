package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
        Idle, TargetAngle
    }

    private final TalonFX             _turretMotor;
    private final MotionMagicVoltage  _turretMotionMagic;
    private final AnalogPotentiometer _turretPotentiometer;
    private final PIDController       _pidController;
    private ControlMode               _controlMode;
    @Logged
    private Angle                     _targetAngleSetpoint;
    @Logged
    private Angle                     _turretAngle;
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

        _turretMotor            = new TalonFX(CANConstants.TURRET_MOTOR);
        _turretPotentiometer    = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER, sensorRange.in(Degrees), sensorOffset.in(Degrees));
        _pidController          = new PIDController(ShooterConstants.TURRET_KP, ShooterConstants.TURRET_KI, ShooterConstants.TURRET_KD);
        _controlMode            = ControlMode.Idle;
        _targetAngleSetpoint    = ShooterConstants.TURRET_HOME_ANGLE;
        _turretAngle            = Degrees.zero();
        _turretSetpoint         = ShooterConstants.TURRET_HOME_ANGLE;
        _commandedTargetAngle   = ShooterConstants.TURRET_HOME_ANGLE;
        _targetAngleError       = Degrees.zero();
        _hasSetpoint            = false;
        _motorVoltage           = Volts.zero();
        _turretMotionMagic      = new MotionMagicVoltage(Degrees.zero());
        _linedUp                = false;
        _motorPositionRotations = 0.0;

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT.in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.Clockwise_Positive;

        _turretMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig));
        _pidController.setTolerance(ShooterConstants.TURRET_TOLERANCE.in(Degrees));

        TalonFXConfiguration config = new TalonFXConfiguration();
        // 1. Set up PID and Feedforward gains
        var slot0Configs = config.Slot0;
        slot0Configs.kP = 4.8;  // Proportional gain (Volts / error in rotations)
        slot0Configs.kI = 0.0;  // Integral gain
        slot0Configs.kD = 0.1;  // Derivative gain
        slot0Configs.kV = 0.12; // Velocity feedforward (Volts / RPS)
        slot0Configs.kA = 0.01; // Acceleration feedforward (Volts / RPS^2)

        // 2. Set Motion Magic constraints (in rotations, RPS, and RPS^2)
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 50.0; // Peak target velocity (RPS)
        motionMagicConfigs.MotionMagicAcceleration   = 100.0;  // Target acceleration (RPS^2)
        motionMagicConfigs.MotionMagicJerk           = 800.0;          // Optional smooth curves (RPS^3)

        // 3. Define mechanical reduction if using internal sensor for mechanism units
        // Example: 10:1 gear ratio. 10 motor turns = 1 mechanism turn
        config.Feedback.SensorToMechanismRatio = 10.0;

        // Apply configuration to the Talon FX hardware
        _turretMotor.getConfigurator().apply(config);
    }

    public void periodic()
    {
        _turretAngle            = Degrees.of(_turretPotentiometer.get()).plus(ShooterConstants.TURRET_POT_OFFSET);
        _motorPositionRotations = _turretMotor.getPosition().getValue().baseUnitMagnitude();
        _motorVoltage           = _turretMotor.getMotorVoltage().getValue();

        switch (_controlMode)
        {
            case TargetAngle:
                _hasSetpoint = true;
                _turretSetpoint = _commandedTargetAngle;
                updateLinedUpState();

                break;

            case Idle:
            default:
                _hasSetpoint = false;
                _linedUp = false;
                _turretSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
                break;
        }

        _turretMotor.setControl(_turretMotionMagic.withPosition(_turretSetpoint));
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

    public boolean isLinedUp()
    {
        return _linedUp;
    }

    public Angle getTargetAngleError()
    {
        return _targetAngleError;
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

    private void updateLinedUpState()
    {
        double angleErrorDegrees   = Math.abs(_turretSetpoint.minus(_turretAngle).in(Degrees));
        double acquireTolerance    = ShooterConstants.TURRET_TOLERANCE.in(Degrees);
        double holdTolerance       = ShooterConstants.TURRET_LINED_UP_HOLD_TOLERANCE.in(Degrees);
        double allowedErrorDegrees = _linedUp ? holdTolerance : acquireTolerance;

        _linedUp = angleErrorDegrees <= allowedErrorDegrees;
    }
}
