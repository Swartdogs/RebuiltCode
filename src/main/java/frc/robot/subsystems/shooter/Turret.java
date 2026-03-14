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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.MeasureUtil;
import frc.robot.util.Utilities;
import limelight.Limelight;
import limelight.networktables.target.AprilTagFiducial;

@Logged
public class Turret
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
    private Angle                            _turretAngle;
    @Logged
    private Angle                            _turretSetpoint;
    @Logged
    private boolean                          _hasSetpoint;
    @Logged
    private Voltage                          _motorVoltage;
    @Logged
    private Distance                         _targetDistance;
    @Logged
    private Pose2d                           _targetPose;
    @Logged
    private boolean                          _limelightHasTarget;
    @Logged
    private boolean                          _targetValid;
    @Logged
    private boolean                          _disabled;

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
        _turretAngle        = Degrees.zero();
        _turretSetpoint     = ShooterConstants.TURRET_HOME_ANGLE;
        _hasSetpoint        = false;
        _motorVoltage       = Volts.zero();
        _targetDistance     = Meters.zero();
        _targetPose         = new Pose2d();
        _limelightHasTarget = false;
        _targetValid        = false;
        _disabled           = false;

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

    public void periodic()
    {
        _currentSwerveState = _swerveStateSupplier.get();

        _turretAngle        = Degrees.of(_turretSensor.get());
        _motorVoltage       = _turretMotor.getMotorVoltage().getValue();
        _limelightHasTarget = false;
        _targetValid        = false;

        var fiducials   = new AprilTagFiducial[0];
        var motorOutput = Volts.zero();
        var rawSetpoint = Degrees.zero();

        switch (_turretState)
        {
            case Track:
                _hasSetpoint = true;
                var results = _limelight.getLatestResults();

                if (results.isPresent())
                {
                    fiducials = results.get().targets_Fiducials;
                }

                var localHubTranslation = getHubTranslationInRobotFrame();
                var turretToHubTranslation = localHubTranslation.minus(ShooterConstants.TURRET_POSITION);

                _targetDistance = Meters.of(turretToHubTranslation.getNorm());
                rawSetpoint = getTurretSetpointForRobotFrameTarget(turretToHubTranslation);

                AprilTagFiducial bestTag = null;

                for (AprilTagFiducial tag : fiducials)
                {
                    if (!Utilities.getOurHubTagIds().contains(tag.fiducialID))
                    {
                        continue;
                    }

                    if (bestTag == null || tag.ta > bestTag.ta || tag.ta == bestTag.ta && Math.abs(tag.tx) < Math.abs(bestTag.tx))
                    {
                        bestTag = tag;
                    }
                }

                if (bestTag != null)
                {
                    _limelightHasTarget = true;
                    var txCorrection = Degrees.of(-bestTag.tx);

                    // Ignore tiny Limelight yaw error so we don't keep hunting on vision noise.
                    if (!MathUtil.isNear(0.0, txCorrection.in(Degrees), ShooterConstants.TURRET_TRACK_TX_DEADBAND.in(Degrees)))
                    {
                        rawSetpoint = rawSetpoint.minus(txCorrection);
                    }
                }

                _targetValid = Utilities.isHubActive();
                break;

            case Pass:
                _hasSetpoint = true;
                _targetValid = true;
                Angle fieldTargetAngle;

                if (Utilities.isBlueAlliance())
                {
                    _targetDistance  = _currentSwerveState.Pose.getMeasureX();
                    fieldTargetAngle = Degrees.of(180);
                }
                else
                {
                    _targetDistance  = GeneralConstants.FIELD_SIZE_X.minus(_currentSwerveState.Pose.getMeasureX());
                    fieldTargetAngle = Degrees.zero();
                }

                rawSetpoint = fieldTargetAngle.minus(_currentSwerveState.Pose.getRotation().getMeasure()).minus(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD);
                break;

            case Idle:
            default:
                _hasSetpoint = false;
                _targetValid = false;
                rawSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
                _targetDistance = Meters.zero();
                break;
        }

        var forwardTranslation    = new Translation2d(_targetDistance, Meters.zero());
        var rotatedByTurretOffset = forwardTranslation.rotateBy(new Rotation2d(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD));
        var rotatedBySetpoint     = rotatedByTurretOffset.rotateBy(new Rotation2d(rawSetpoint));

        _targetPose     = _currentSwerveState.Pose.plus(new Transform2d(rotatedBySetpoint, new Rotation2d()));
        _turretSetpoint = MeasureUtil.clamp(moduloAngle(rawSetpoint), ShooterConstants.TURRET_SOFT_MIN_ANGLE, ShooterConstants.TURRET_SOFT_MAX_ANGLE);

        if (_hasSetpoint && !_disabled)
        {
            motorOutput = Volts.of(_pidController.calculate(_turretAngle.in(Degrees), _turretSetpoint.in(Degrees)));
        }

        motorOutput = applySoftLimit(motorOutput);
        _turretMotor.setVoltage(motorOutput.in(Volts));
    }

    public void simulationPeriodic()
    {
    }

    public Distance getTargetDistance()
    {
        return _targetDistance;
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

    public boolean hasValidTarget()
    {
        return _targetValid;
    }

    public boolean hasLimelightTarget()
    {
        return _limelightHasTarget;
    }

    public boolean isReadyToShoot()
    {
        return hasValidTarget() && isLinedUp();
    }

    public void setDisabled(boolean disabled)
    {
        _disabled = disabled;
    }

    private void updateFilter(List<Integer> filters)
    {
        if (DriverStation.isDisabled())
        {
            _limelight.getSettings().withAprilTagIdFilter(filters).save();
        }
    }

    private Angle moduloAngle(Angle angle)
    {
        var degrees = angle.in(Degrees);

        return Degrees.of(MathUtil.inputModulus(degrees, -180, 180));
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

    private Translation2d getHubTranslationInRobotFrame()
    {
        var robotToHubField = Utilities.getHubCoordinates().minus(_currentSwerveState.Pose.getTranslation());

        return robotToHubField.rotateBy(_currentSwerveState.Pose.getRotation().unaryMinus());
    }

    private Angle getTurretSetpointForRobotFrameTarget(Translation2d targetTranslation)
    {
        return targetTranslation.getAngle().getMeasure().minus(ShooterConstants.TURRET_ZERO_OFFSET_FROM_ROBOT_FORWARD);
    }
}
