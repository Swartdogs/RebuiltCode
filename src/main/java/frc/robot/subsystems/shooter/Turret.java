package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.MeasureUtil;
import limelight.Limelight;

@Logged
public class Turret extends SubsystemBase
{
    public enum TurretState
    {
        Idle, Track, Pass
    }

    private final TalonFX                    _turretMotor;
    private final AnalogPotentiometer        _turretSensor;
    private final Supplier<SwerveDriveState> _swerveStateSupplier;
    private final Limelight                  _limelight;
    private final PositionVoltage            _positionRequest;
    private SwerveDriveState                 _currentSwerveState;
    private TurretState                      _turretState;
    private List<Integer>                    _activeTagFilter;
    @Logged
    private Angle                            _fieldAngle;
    @Logged
    private Angle                            _robotAngle;
    @Logged
    private Angle                            _turretSetpoint;
    @Logged
    private boolean                          _hasSetpoint;
    @Logged
    private boolean                          _hasTarget;
    @Logged
    private Voltage                          _motorVoltage;

    public Turret(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        _swerveStateSupplier = swerveStateSupplier;

        var sensorRange  = ShooterConstants.TURRET_HARD_MAX_ANGLE.minus(ShooterConstants.TURRET_HARD_MIN_ANGLE);
        var sensorOffset = ShooterConstants.TURRET_HARD_MIN_ANGLE;

        if (ShooterConstants.TURRET_SENSOR_INVERTED)
        {
            sensorRange  = sensorRange.unaryMinus();
            sensorOffset = sensorOffset.unaryMinus();
        }

        var sensorInput = new AnalogInput(AIOConstants.TURRET_POTENTIOMETER);
        _turretSensor = new AnalogPotentiometer(sensorInput, sensorRange.in(Degrees), sensorOffset.in(Degrees));

        _turretMotor = new TalonFX(CANConstants.TURRET_MOTOR);

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT.in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.Clockwise_Positive;

        var slot0 = new Slot0Configs();
        slot0.kP = ShooterConstants.TURRET_KP;
        slot0.kI = ShooterConstants.TURRET_KI;
        slot0.kD = ShooterConstants.TURRET_KD;

        _turretMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig).withSlot0(slot0));

        _positionRequest = new PositionVoltage(0).withSlot(0);

        _limelight = new Limelight(ShooterConstants.LIMELIGHT_NAME);

        _currentSwerveState = new SwerveDriveState();
        _turretState        = TurretState.Idle;
        _activeTagFilter    = List.of();
        _robotAngle         = Degrees.zero();
        _fieldAngle         = Degrees.zero();
        _turretSetpoint     = ShooterConstants.TURRET_HOME_ANGLE;
        _hasSetpoint        = false;
        _hasTarget          = false;
        _motorVoltage       = Volts.zero();
    }

    public Command stop()
    {
        return runOnce(() ->
        {
            _turretState    = TurretState.Idle;
            _hasSetpoint    = false;
            _turretSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
            _turretMotor.setVoltage(0.0);
        });
    }

    @Override
    public void periodic()
    {
        var latestSwerveState = _swerveStateSupplier.get();
        if (latestSwerveState != null)
        {
            _currentSwerveState = latestSwerveState;
        }

        _robotAngle   = Degrees.of(_turretSensor.get());
        _fieldAngle   = toFieldFrame(_robotAngle);
        _motorVoltage = _turretMotor.getMotorVoltage().getValue();

        updateVisionFilter();

        var requestedSetpoint = getRequestedSetpoint();

        if (requestedSetpoint == null)
        {
            _hasSetpoint    = false;
            _turretSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
            _turretMotor.setVoltage(0.0);
        }
        else
        {
            _hasSetpoint    = true;
            _turretSetpoint = MeasureUtil.clamp(requestedSetpoint, ShooterConstants.TURRET_SOFT_MIN_ANGLE, ShooterConstants.TURRET_SOFT_MAX_ANGLE);

            var target = _turretSetpoint.times(ShooterConstants.TURRET_GEAR_RATIO);
            _turretMotor.setControl(_positionRequest.withPosition(target.in(Rotations)));
        }
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
        if (!_hasSetpoint)
        {
            return _turretState == TurretState.Idle;
        }

        if (_turretState == TurretState.Track && !_hasTarget)
        {
            return false;
        }

        return _robotAngle.isNear(_turretSetpoint, ShooterConstants.TURRET_TOLERANCE);
    }

    private Angle getRequestedSetpoint()
    {
        switch (_turretState)
        {
            case Idle:
                _hasTarget = false;
                return null;

            case Pass:
                _hasTarget = false;
                return TurretDirector.passTargetRobotFrame(_currentSwerveState);

            case Track:
                var latestResults = _limelight.getLatestResults();
                if (latestResults.isEmpty())
                {
                    _hasTarget = false;
                    return ShooterConstants.TURRET_HOME_ANGLE;
                }

                var avgPose = TurretDirector.averageCenterTagPose(latestResults.get().targets_Fiducials);
                _hasTarget = avgPose != null;

                if (_hasTarget)
                {
                    return TurretDirector.calculateTrackSetpoint(_robotAngle, avgPose);
                }

                return ShooterConstants.TURRET_HOME_ANGLE;
        }

        return null;
    }

    private void updateVisionFilter()
    {
        if (!DriverStation.isDisabled())
        {
            return;
        }

        updateAllianceTagFilter();
    }

    private void updateAllianceTagFilter()
    {
        var tagFilter = TurretDirector.getAllianceCenterTags();
        if (_activeTagFilter.equals(tagFilter))
        {
            return;
        }

        var tagOffset = new Translation3d(ShooterConstants.TURRET_CENTER_TAG_TO_HUB_CENTER, Inches.zero(), Inches.zero());
        _limelight.getSettings().withAprilTagIdFilter(tagFilter).withAprilTagOffset(tagOffset).save();
        _activeTagFilter = List.copyOf(tagFilter);
    }

    private Angle toFieldFrame(Angle robotFrameAngle)
    {
        return robotFrameAngle.plus(_currentSwerveState.Pose.getRotation().getMeasure());
    }
}
