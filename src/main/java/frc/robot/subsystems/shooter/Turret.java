package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Utilities;
import limelight.Limelight;

@Logged
public class Turret extends SubsystemBase
{
    public enum TurretState
    {
        Idle, Track, Pass
    }

    private TalonFX                    _turretMotor;
    private TalonFXSimState            _turretSimMotor;
    private DCMotorSim                 _motorSimModel;
    private AnalogInput                _turretSensorInput;
    private AnalogPotentiometer        _turretSensor;
    private AnalogInputSim             _turretSimSensor;
    private Limelight                  _limelight;
    @Logged
    private Angle                      _robotTurretAngle;
    @Logged
    private Angle                      _turretSetpoint;
    @Logged
    private Angle                      _fieldTurretAngle;
    @Logged
    private Voltage                    _turretMotorVoltage;
    @Logged
    private TurretState                _turretState;
    @Logged
    private boolean                    _hasTarget;
    @Logged
    private Angle                      _targetHorizontalOffset;
    private Supplier<SwerveDriveState> _swerveStateSupplier;
    private SwerveDriveState           _swerveDriveState;
    private PositionVoltage            _positionRequest = new PositionVoltage(0).withSlot(0);
    private List<Double>               _cachedTagFilter;

    public Turret()
    {
        this(null);
    }

    public Turret(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        // Setting up motor
        _turretMotor = new TalonFX(CANConstants.TURRET_MOTOR);
        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT;
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.CounterClockwise_Positive;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = ShooterConstants.TURRET_KP;
        slot0Configs.kI = ShooterConstants.TURRET_KI;
        slot0Configs.kD = ShooterConstants.TURRET_KD;

        _turretMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig).withSlot0(slot0Configs));
        _turretSensorInput = new AnalogInput(AIOConstants.TURRET_POTENTIOMETER);
        _turretSensor      = new AnalogPotentiometer(_turretSensorInput, ShooterConstants.TURRET_MAX_ANGLE - ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MIN_ANGLE);
        _positionRequest   = new PositionVoltage(0).withSlot(0);
        _limelight         = new Limelight(ShooterConstants.LIMELIGHT_NAME);

        if (RobotBase.isReal())
        {
            _turretSimMotor  = null;
            _motorSimModel   = null;
            _turretSimSensor = null;
        }
        else
        {
            _turretSimMotor  = _turretMotor.getSimState();
            _turretSimSensor = new AnalogInputSim(_turretSensorInput);
            var gearbox = DCMotor.getKrakenX44Foc(1);
            _motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, ShooterConstants.TURRET_GEAR_RATIO), gearbox);
        }
        _robotTurretAngle       = Degrees.of(0.0);
        _fieldTurretAngle       = Degrees.of(0.0);
        _turretMotorVoltage     = Volts.of(0.0);
        _turretState            = TurretState.Idle;
        _hasTarget              = false;
        _targetHorizontalOffset = Degrees.of(0.0);
        _turretSetpoint         = null;
        _swerveStateSupplier    = (swerveStateSupplier == null) ? () -> new SwerveDriveState() : swerveStateSupplier;
        _swerveDriveState       = new SwerveDriveState();
        _cachedTagFilter        = List.of();
    }

    @Override
    public void periodic()
    {
        _robotTurretAngle = Degrees.of(_turretSensor.get());

        SwerveDriveState state = _swerveStateSupplier.get();
        if (state != null)
        {
            _swerveDriveState = state;
        }
        _fieldTurretAngle   = _robotTurretAngle.plus(_swerveDriveState.Pose.getRotation().getMeasure());
        _turretMotorVoltage = _turretMotor.getMotorVoltage().getValue();

        if (RobotBase.isReal())
        {
            List<Double> desiredTags = Utilities.getOurHubTagIds();
            if (!_cachedTagFilter.equals(desiredTags))
            {
                _limelight.getSettings().withArilTagIdFilter(desiredTags).save();
                _cachedTagFilter = List.copyOf(desiredTags);
            }

            var targetData = _limelight.getData().targetData;
            _hasTarget              = targetData.getTargetStatus();
            _targetHorizontalOffset = Degrees.of(targetData.getHorizontalOffset());
        }
        else
        {
            _hasTarget              = false;
            _targetHorizontalOffset = Degrees.of(0.0);
        }

        Angle requestedSetpoint = switch (_turretState)
        {
            case Idle -> null;
            case Track -> _hasTarget ? _robotTurretAngle.plus(_targetHorizontalOffset) : Degrees.of(ShooterConstants.TURRET_HOME_ANGLE);
            case Pass -> ShooterConstants.TURRET_PASS_TARGET.minus(_swerveDriveState.Pose.getRotation().getMeasure());
        };

        if (requestedSetpoint == null)
        {
            _turretSetpoint = null;
            _turretMotor.setVoltage(0.0);
            return;
        }

        _turretSetpoint = clampToTurretLimits(requestedSetpoint);
        double targetMotorRotations = _turretSetpoint.in(Degrees) / 360.0 * ShooterConstants.TURRET_GEAR_RATIO;
        _turretMotor.setControl(_positionRequest.withPosition(targetMotorRotations));
    }

    @Override
    public void simulationPeriodic()
    {
        if (_turretSimMotor == null || _motorSimModel == null || _turretSimSensor == null) return;

        _turretSimMotor.setSupplyVoltage(RoboRioSim.getVInVoltage());

        Voltage motorVoltage = _turretSimMotor.getMotorVoltageMeasure();
        _motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        _motorSimModel.update(GeneralConstants.LOOP_PERIOD_SECS);

        _turretSimMotor.setRawRotorPosition(_motorSimModel.getAngularPosition().times(ShooterConstants.TURRET_GEAR_RATIO));
        _turretSimMotor.setRotorVelocity(_motorSimModel.getAngularVelocity().times(ShooterConstants.TURRET_GEAR_RATIO));

        double mechanismAngleDeg = _motorSimModel.getAngularPosition().in(Degrees) / ShooterConstants.TURRET_GEAR_RATIO;
        double clampedAngleDeg   = Math.max(ShooterConstants.TURRET_MIN_ANGLE, Math.min(ShooterConstants.TURRET_MAX_ANGLE, mechanismAngleDeg));
        double normalized        = (clampedAngleDeg - ShooterConstants.TURRET_MIN_ANGLE) / (ShooterConstants.TURRET_MAX_ANGLE - ShooterConstants.TURRET_MIN_ANGLE);
        _turretSimSensor.setVoltage(RoboRioSim.getUserVoltage5V() * normalized);
    }

    public void setTurretState(TurretState state)
    {
        _turretState = state;
    }

    public TurretState getTurretState()
    {
        return _turretState;
    }

    public boolean hasTarget()
    {
        return _hasTarget;
    }

    public boolean isLinedUp()
    {
        if (_turretSetpoint == null) return _turretState == TurretState.Idle;
        if (_turretState == TurretState.Track && !_hasTarget) return false;

        double errorDeg = Math.abs(_robotTurretAngle.in(Degrees) - _turretSetpoint.in(Degrees));
        return errorDeg <= ShooterConstants.TURRET_TOLERANCE;
    }

    public double getDistanceToTarget()
    {
        if (RobotBase.isSimulation() || !_hasTarget)
        {
            return 0.0;
        }

        var targetPose = _limelight.getData().targetData.getCameraToTarget();
        return targetPose.getTranslation().toTranslation2d().getNorm();
    }

    private static Angle clampToTurretLimits(Angle angle)
    {
        double clampedDegrees = MathUtil.clamp(angle.in(Degrees), ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE);
        return Degrees.of(clampedDegrees);
    }
}
