package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
import frc.robot.util.MeasureUtil;
import frc.robot.util.Utilities;
import limelight.Limelight;

@Logged
public class Turret extends SubsystemBase
{
    public enum TurretState
    {
        Idle, Track, Pass
    }

    private final TalonFX              _turretMotor;
    private final TalonFXSimState      _turretMotorSim;
    private final DCMotorSim           _motorSimModel;
    private final AnalogPotentiometer  _turretSensor;
    private final AnalogInputSim       _turretSensorSim;
    private final Limelight            _limelight;
    private Supplier<SwerveDriveState> _swerveStateSupplier;
    private SwerveDriveState           _swerveDriveState;
    private PositionVoltage            _positionRequest = new PositionVoltage(0).withSlot(0);
    private List<Integer>              _cachedTagFilter;
    @Logged
    private Angle                      _fieldTurretAngle;
    @Logged
    private Angle                      _robotTurretAngle;
    @Logged
    private Angle                      _turretSetpoint;
    @Logged
    private boolean                    _hasSetpoint;
    @Logged
    private Voltage                    _turretMotorVoltage;
    @Logged
    private TurretState                _turretState;
    @Logged
    private boolean                    _hasTarget;
    @Logged
    private Angle                      _targetHorizontalOffset;
    @Logged
    private Angle                      _predictedRobotHeading;
    @Logged
    private Angle                      _rotationLookahead;

    public Turret(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        // Setting up motor
        _turretMotor = new TalonFX(CANConstants.TURRET_MOTOR);
        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT.in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.CounterClockwise_Positive;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = ShooterConstants.TURRET_KP;
        slot0Configs.kI = ShooterConstants.TURRET_KI;
        slot0Configs.kD = ShooterConstants.TURRET_KD;

        _turretMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig).withSlot0(slot0Configs));
        AnalogInput turretSensorInput = new AnalogInput(AIOConstants.TURRET_POTENTIOMETER);
        _turretSensor    = new AnalogPotentiometer(turretSensorInput, ShooterConstants.TURRET_MAX_ANGLE.minus(ShooterConstants.TURRET_MIN_ANGLE).in(Degrees), ShooterConstants.TURRET_MIN_ANGLE.in(Degrees));
        _positionRequest = new PositionVoltage(0).withSlot(0);
        _limelight       = new Limelight(ShooterConstants.LIMELIGHT_NAME);

        _robotTurretAngle       = Degrees.of(0.0);
        _fieldTurretAngle       = Degrees.of(0.0);
        _turretMotorVoltage     = Volts.of(0.0);
        _turretState            = TurretState.Idle;
        _hasTarget              = false;
        _targetHorizontalOffset = Degrees.of(0.0);
        _predictedRobotHeading  = Degrees.zero();
        _rotationLookahead      = Degrees.zero();
        _turretSetpoint         = Degrees.zero();
        _hasSetpoint            = false;
        _swerveStateSupplier    = (swerveStateSupplier == null) ? () -> new SwerveDriveState() : swerveStateSupplier;
        _swerveDriveState       = new SwerveDriveState();
        _cachedTagFilter        = List.of();

        if (RobotBase.isReal())
        {
            _turretMotorSim  = null;
            _motorSimModel   = null;
            _turretSensorSim = null;
        }
        else
        {
            _turretMotorSim  = _turretMotor.getSimState();
            _turretSensorSim = new AnalogInputSim(turretSensorInput);
            var gearbox = DCMotor.getKrakenX44(1);
            _motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, ShooterConstants.TURRET_GEAR_RATIO.in(Value)), gearbox);
        }
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
        var robotHeading = _swerveDriveState.Pose.getRotation().getMeasure();
        _fieldTurretAngle      = _robotTurretAngle.plus(robotHeading);
        _rotationLookahead     = Degrees.of(Math.toDegrees(_swerveDriveState.Speeds.omegaRadiansPerSecond * ShooterConstants.TURRET_LOOKAHEAD.in(Seconds)));
        _predictedRobotHeading = robotHeading.plus(_rotationLookahead);
        _turretMotorVoltage    = _turretMotor.getMotorVoltage().getValue();

        if (RobotBase.isReal())
        {
            List<Integer> desiredTags = Utilities.getOurHubTagIds();
            if (!_cachedTagFilter.equals(desiredTags))
            {
                _limelight.getSettings().withAprilTagIdFilter(desiredTags).save();
                _cachedTagFilter = List.copyOf(desiredTags);
            }

            var targetData = _limelight.getData().targetData;
            _hasTarget              = targetData.getTargetStatus();
            _targetHorizontalOffset = Degrees.of(targetData.getHorizontalOffset());
        }

        Angle requestedSetpoint = switch (_turretState)
        {
            case Idle -> null;
            case Track -> _hasTarget ? _robotTurretAngle.plus(_targetHorizontalOffset).minus(_rotationLookahead) : ShooterConstants.TURRET_HOME_ANGLE;
            case Pass -> ShooterConstants.TURRET_PASS_TARGET.minus(_predictedRobotHeading);
        };

        if (requestedSetpoint == null)
        {
            _hasSetpoint    = false;
            _turretSetpoint = _robotTurretAngle;
            _turretMotor.setVoltage(0.0);
            return;
        }

        _hasSetpoint    = true;
        _turretSetpoint = MeasureUtil.clamp(requestedSetpoint, ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE);
        var target = _turretSetpoint.times(ShooterConstants.TURRET_GEAR_RATIO);
        _turretMotor.setControl(_positionRequest.withPosition(target.in(Rotations)));
    }

    @Override
    public void simulationPeriodic()
    {
        _hasTarget              = false;
        _targetHorizontalOffset = Degrees.of(0.0);

        _turretMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

        Voltage motorVoltage = _turretMotorSim.getMotorVoltageMeasure();
        _motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        _motorSimModel.update(GeneralConstants.LOOP_PERIOD.in(Seconds));

        _turretMotorSim.setRawRotorPosition(_motorSimModel.getAngularPosition().times(ShooterConstants.TURRET_GEAR_RATIO));
        _turretMotorSim.setRotorVelocity(_motorSimModel.getAngularVelocity().times(ShooterConstants.TURRET_GEAR_RATIO));

        var   mechanismAngle = _motorSimModel.getAngularPosition().div(ShooterConstants.TURRET_GEAR_RATIO);
        Angle clamped        = MeasureUtil.clamp(mechanismAngle, ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE);
        var   normalized     = clamped.minus(ShooterConstants.TURRET_MIN_ANGLE).div(ShooterConstants.TURRET_MAX_ANGLE.minus(ShooterConstants.TURRET_MIN_ANGLE));
        _turretSensorSim.setVoltage(RoboRioSim.getUserVoltage5V() * normalized.in(Value));
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
        if (!_hasSetpoint) return _turretState == TurretState.Idle;
        if (_turretState == TurretState.Track && !_hasTarget) return false;

        return _robotTurretAngle.isNear(_turretSetpoint, ShooterConstants.TURRET_TOLERANCE);
    }

    public Distance getDistanceToTarget()
    {
        if (RobotBase.isSimulation() || !_hasTarget)
        {
            return Meters.zero();
        }

        var targetPose = _limelight.getData().targetData.getCameraToTarget();
        return Meters.of(targetPose.getTranslation().toTranslation2d().getNorm());
    }
}
