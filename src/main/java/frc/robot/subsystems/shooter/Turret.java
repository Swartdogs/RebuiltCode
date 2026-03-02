package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.MeasureUtil;
import frc.robot.util.Utilities;
import limelight.Limelight;
import limelight.networktables.target.AprilTagFiducial;

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
    private final PIDController              _pidController;
    private SwerveDriveState                 _currentSwerveState;
    private TurretState                      _turretState;
    @Logged
    private Angle                            _fieldAngle;
    @Logged
    private Angle                            _robotAngle;
    @Logged
    private Angle                            _turretSetpoint;
    @Logged
    private boolean                          _hasSetpoint;
    @Logged
    private Voltage                          _motorVoltage;
    @Logged
    private Distance                         _hubDistance;
    @Logged
    private Pose2d                           _targetPose;
    @Logged
    private Transform2d                      _targetTransform = new Transform2d();

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

        _turretMotor        = new TalonFX(CANConstants.TURRET_MOTOR);
        _turretSensor       = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER, sensorRange.in(Degrees), sensorOffset.in(Degrees));
        _limelight          = new Limelight(ShooterConstants.LIMELIGHT_NAME);
        _pidController      = new PIDController(ShooterConstants.TURRET_KP, ShooterConstants.TURRET_KI, ShooterConstants.TURRET_KD);
        _currentSwerveState = new SwerveDriveState();
        _turretState        = TurretState.Idle;
        _robotAngle         = Degrees.zero();
        _fieldAngle         = Degrees.zero();
        _turretSetpoint     = ShooterConstants.TURRET_HOME_ANGLE;
        _hasSetpoint        = false;
        _motorVoltage       = Volts.zero();
        _hubDistance        = Meters.zero();
        _targetPose         = new Pose2d();

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT.in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.Clockwise_Positive;

        _turretMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig));

        _pidController.setTolerance(ShooterConstants.TURRET_TOLERANCE.in(Degrees));
        _pidController.setSetpoint(ShooterConstants.TURRET_HOME_ANGLE.in(Degrees));

        _limelight.getSettings().withAprilTagOffset(ShooterConstants.CENTER_TAG_TO_HUB_CENTER_OFFSET).withAprilTagIdFilter(ShooterConstants.RED_HUB_TAG_IDS).save();

        var blueTrigger  = new Trigger(Utilities::isBlueAlliance);
        var redTrigger   = new Trigger(Utilities::isRedAlliance);
        var emptyTrigger = new Trigger(() -> DriverStation.getAlliance().isEmpty());

        blueTrigger.onTrue(Commands.runOnce(() -> updateFilter(ShooterConstants.BLUE_HUB_TAG_IDS)));
        redTrigger.onTrue(Commands.runOnce(() -> updateFilter(ShooterConstants.RED_HUB_TAG_IDS)));
        emptyTrigger.onTrue(Commands.runOnce(() -> updateFilter(ShooterConstants.ALL_HUB_TAG_IDS)));
    }

    @Override
    public void periodic()
    {
        _currentSwerveState = _swerveStateSupplier.get();

        _robotAngle   = Degrees.of(_turretSensor.get());
        _fieldAngle   = toFieldFrame(_robotAngle);
        _motorVoltage = _turretMotor.getMotorVoltage().getValue();

        var fiducials   = new AprilTagFiducial[0];
        var motorOutput = Volts.zero();

        switch (_turretState)
        {
            case Track:
                var results = _limelight.getLatestResults();

                if (results.isPresent())
                {
                    fiducials = results.get().targets_Fiducials;
                }
            case Pass:
                _hasSetpoint = true;
                var directorResult = TurretDirector.calculate(_turretState, _currentSwerveState, fiducials);

                _targetPose = _currentSwerveState.Pose.plus(new Transform2d(directorResult, directorResult.getAngle()));

                _hubDistance = Meters.of(directorResult.getNorm());
                _turretSetpoint = MeasureUtil.clamp(directorResult.getAngle().getMeasure().minus(ShooterConstants.HUB_ZERO_OFFSET_FROM_ROBOT_FORWARD), ShooterConstants.TURRET_SOFT_MIN_ANGLE, ShooterConstants.TURRET_SOFT_MAX_ANGLE);
                break;

            case Idle:
            default:
                _hasSetpoint = false;
                _turretSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
                _hubDistance = Meters.zero();
                break;
        }

        if (_hasSetpoint)
        {
            motorOutput = Volts.of(_pidController.calculate(_robotAngle.in(Degrees), _turretSetpoint.in(Degrees)));
        }

        _turretMotor.setVoltage(motorOutput.in(Volts));
    }

    public Distance getHubDistance()
    {
        return _hubDistance;
    }

    public void setTurretState(TurretState state)
    {
        _turretState = state;
    }

    public TurretState getTurretState()
    {
        return _turretState;
    }

    public boolean isLinedUp()
    {
        return _hasSetpoint && _pidController.atSetpoint();
    }

    public Command getTrackCmd()
    {
        return startEnd(() -> setTurretState(TurretState.Track), () -> setTurretState(TurretState.Idle));
    }

    public Command getPassCmd()
    {
        return startEnd(() -> setTurretState(TurretState.Pass), () -> setTurretState(TurretState.Idle));
    }

    public Command getIdleCmd()
    {
        return runOnce(() -> setTurretState(TurretState.Idle));
    }

    private Angle toFieldFrame(Angle robotFrameAngle)
    {
        return robotFrameAngle.plus(_currentSwerveState.Pose.getRotation().getMeasure()).plus(ShooterConstants.HUB_ZERO_OFFSET_FROM_ROBOT_FORWARD);
    }

    private Angle toRobotFrame(Angle fieldFrameAngle)
    {
        return fieldFrameAngle.minus(_currentSwerveState.Pose.getRotation().getMeasure()).minus(ShooterConstants.HUB_ZERO_OFFSET_FROM_ROBOT_FORWARD);
    }

    private void updateFilter(List<Integer> filters)
    {
        if (DriverStation.isDisabled())
    {
        _limelight.getSettings().withAprilTagIdFilter(filters).save();
        }
    }
}
