package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.MeasureUtil;
import frc.robot.util.Utilities;
import limelight.Limelight;
import limelight.results.RawFiducial;

@Logged
public class Turret extends SubsystemBase
{
    public enum TurretState
    {
        Idle, Track, Pass
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

    private final TalonFX                    _turretMotor;
    private final TalonFXSimState            _turretMotorSim;
    private final DCMotorSim                 _motorSimModel;
    private final AnalogInput                _turretSensorInput;
    private final AnalogPotentiometer        _turretSensor;
    private final AnalogInputSim             _turretSensorSim;
    private final Limelight                  _limelight;
    private final Supplier<SwerveDriveState> _swerveStateSupplier;
    private final PositionVoltage            _positionRequest = new PositionVoltage(0).withSlot(0);
    private List<Integer>                    _cachedTagFilter;
    private final Angle                      _defaultSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
    private SwerveDriveState                 _swerveDriveState;
    @Logged
    private Angle                            _fieldTurretAngle;
    @Logged
    private Angle                            _robotTurretAngle;
    @Logged
    private Angle                            _continuousRobotAngle;
    @Logged
    private Angle                            _continuousTurretSetpoint;
    @Logged
    private Angle                            _turretSetpoint;
    @Logged
    private boolean                          _hasSetpoint;
    @Logged
    private Voltage                          _turretMotorVoltage;
    @Logged
    private TurretState                      _turretState;
    @Logged
    private boolean                          _hasTarget;
    @Logged
    private Angle                            _targetHorizontalOffset;
    @Logged
    private Distance                         _distanceToHub;
    private TurretDirector.TagObservation    _centerTagObservation;
    private TurretDirector.TagObservation    _offsetTagObservation;
    private Angle                            _lastWrappedRobotAngle;

    public Turret(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        _turretMotor         = new TalonFX(CANConstants.TURRET_MOTOR);
        _swerveStateSupplier = swerveStateSupplier;
        _swerveDriveState    = new SwerveDriveState();

        _fieldTurretAngle         = Degrees.zero();
        _robotTurretAngle         = Degrees.zero();
        _turretSetpoint           = _defaultSetpoint;
        _hasSetpoint              = false;
        _turretMotorVoltage       = Volts.zero();
        _turretState              = TurretState.Idle;
        _hasTarget                = false;
        _targetHorizontalOffset   = Degrees.zero();
        _distanceToHub            = Meters.zero();
        _cachedTagFilter          = List.of();
        _centerTagObservation     = null;
        _offsetTagObservation     = null;
        _continuousRobotAngle     = Degrees.zero();
        _continuousTurretSetpoint = Degrees.zero();
        _lastWrappedRobotAngle    = Degrees.zero();

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

        _turretSensorInput = new AnalogInput(AIOConstants.TURRET_POTENTIOMETER);
        _turretSensor      = new AnalogPotentiometer(_turretSensorInput, getTurretPotFullRange().in(Degrees), getTurretPotOffset().in(Degrees));
        syncMotorEncoderToPotentiometer();

        if (RobotBase.isReal())
        {
            _limelight       = new Limelight(ShooterConstants.LIMELIGHT_NAME);
            _turretMotorSim  = null;
            _motorSimModel   = null;
            _turretSensorSim = null;
        }
        else
        {
            _limelight       = null;
            _turretMotorSim  = _turretMotor.getSimState();
            _turretSensorSim = new AnalogInputSim(_turretSensorInput);
            var gearbox = DCMotor.getKrakenX44(1);
            _motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, ShooterConstants.TURRET_GEAR_RATIO.in(Value)), gearbox);
        }
    }

    /**
     * This function executes automatically every 20 milliseconds. For the turret
     * specifically, we need to ensure the turret is behaving according to the rules
     * defined by the state the turret is in. If in <code>Idle</code>, the turret
     * shouldn't rotate at all, even if the robot rotates. If in <code>Track</code>
     * or <code>Pass</code>, the turret should rotate to whatever angle is provided
     * by the <code>
     * TurretDirector</code> class.
     */
    @Override
    public void periodic()
    {
        SwerveDriveState state = _swerveStateSupplier == null ? null : _swerveStateSupplier.get();
        if (state != null)
        {
            _swerveDriveState = state;
        }

        Angle robotHeading = _swerveDriveState.Pose.getRotation().getMeasure();
        _robotTurretAngle = getCalibratedRobotAngle();
        updateContinuousRobotAngle(_robotTurretAngle);
        _fieldTurretAngle   = _robotTurretAngle.plus(robotHeading);
        _turretMotorVoltage = _turretMotor.getMotorVoltage().getValue();

        updateVisionData();

        var                              directorContext = new TurretDirector.DirectorContext(_turretState, _fieldTurretAngle, robotHeading, _targetHorizontalOffset, _hasTarget, _centerTagObservation, _offsetTagObservation);
        TurretDirector.TurretAimSolution aimSolution     = TurretDirector.getAimSolution(directorContext);

        if (!aimSolution.hasSetpoint())
        {
            stopTurret();
            return;
        }

        _distanceToHub = Meters.of(aimSolution.distanceToHubMeters());
        Angle requestedRobotSetpoint = clampToTurretLimits(aimSolution.fieldSetpoint().minus(robotHeading));
        _continuousTurretSetpoint = chooseContinuousSetpoint(requestedRobotSetpoint);
        _turretSetpoint           = wrap(_continuousTurretSetpoint);
        _hasSetpoint              = true;

        double targetMotorRotations = _continuousTurretSetpoint.times(ShooterConstants.TURRET_GEAR_RATIO).in(Rotations);
        _turretMotor.setControl(_positionRequest.withPosition(targetMotorRotations));
    }

    /**
     * Any logic needed to specifically manage any hardware interactions when the
     * robot code is executing in the simulation (for example, update the 10-turn
     * potentiometer's angle based on how the motor driving the turret is moving)
     * should go here.
     */
    @Override
    public void simulationPeriodic()
    {
        _turretMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

        Voltage motorVoltage = _turretMotorSim.getMotorVoltageMeasure();
        _motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        _motorSimModel.update(GeneralConstants.LOOP_PERIOD.in(Seconds));

        _turretMotorSim.setRawRotorPosition(_motorSimModel.getAngularPosition().times(ShooterConstants.TURRET_GEAR_RATIO));
        _turretMotorSim.setRotorVelocity(_motorSimModel.getAngularVelocity().times(ShooterConstants.TURRET_GEAR_RATIO));

        Angle         mechanismAngle = _motorSimModel.getAngularPosition();
        Angle         clampedAngle   = MeasureUtil.clamp(mechanismAngle, ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE);
        Dimensionless normalized     = clampedAngle.minus(ShooterConstants.TURRET_MIN_ANGLE).div(ShooterConstants.TURRET_MAX_ANGLE.minus(ShooterConstants.TURRET_MIN_ANGLE));
        Voltage       sensorVoltage  = ShooterConstants.TURRET_POTENTIOMETER_MIN_VOLTS.plus(normalized.times(ShooterConstants.TURRET_POTENTIOMETER_MAX_VOLTS.minus(ShooterConstants.TURRET_POTENTIOMETER_MIN_VOLTS)));
        _turretSensorSim.setVoltage(sensorVoltage.in(Volts));
    }

    /**
     * Sets the desired state of the turret.
     *
     * @param state The desired state of the turret
     */
    public void setTurretState(TurretState state)
    {
        _turretState = state == null ? TurretState.Idle : state;
    }

    /**
     * Gets the current state of the turret.
     *
     * @return The current state of the turret
     */
    public TurretState getTurretState()
    {
        return _turretState;
    }

    /**
     * Gets the current angle of the turret. Angle is given in field-relative space.
     *
     * @return The current angle of the turret
     */
    public Angle getAngle()
    {
        return _fieldTurretAngle;
    }

    /**
     * Gets whether the turret is currently pointing at its desired target. The
     * target to point at is determined by the state of the turret.
     *
     * @return <code>true</code> if the turret is facing the desired target location
     *         (or if there is no desired location), otherwise <code>false</code>.
     */
    public boolean isLinedUp()
    {
        if (!_hasSetpoint)
        {
            return false;
        }

        if (_turretState == TurretState.Track && !_hasTarget)
        {
            return false;
        }

        return _continuousRobotAngle.isNear(_continuousTurretSetpoint, ShooterConstants.TURRET_TOLERANCE);
    }

    public boolean hasTarget()
    {
        return _hasTarget;
    }

    public Distance getDistanceToTarget()
    {
        if (_distanceToHub.gt(Meters.zero()))
        {
            return _distanceToHub;
        }

        if (_limelight == null || !_hasTarget)
        {
            return Meters.zero();
        }

        var targetPose = _limelight.getData().targetData.getCameraToTarget();
        return Meters.of(targetPose.getTranslation().toTranslation2d().getNorm());
    }

    private void updateVisionData()
    {
        if (_limelight == null)
        {
            _hasTarget              = false;
            _targetHorizontalOffset = Degrees.zero();
            _centerTagObservation   = null;
            _offsetTagObservation   = null;
            return;
        }

        List<Integer> desiredTags = Utilities.getOurHubTagIds();
        if (!_cachedTagFilter.equals(desiredTags))
        {
            _limelight.getSettings().withAprilTagIdFilter(desiredTags).save();
            _cachedTagFilter = List.copyOf(desiredTags);
        }

        var limelightData = _limelight.getData();
        var targetData    = limelightData.targetData;
        _targetHorizontalOffset = Degrees.of(targetData.getHorizontalOffset());

        boolean                                     allianceIsRed    = Utilities.isRedAlliance();
        List<List<Integer>>                         hubPairs         = TurretDirector.getHubCenterOffsetPairs(allianceIsRed);
        Map<Integer, TurretDirector.TagObservation> observationsById = new HashMap<>();
        for (RawFiducial fiducial : limelightData.getRawFiducials())
        {
            observationsById.put(fiducial.id, new TurretDirector.TagObservation(fiducial.id, Degrees.of(fiducial.txnc), fiducial.distToRobot));
        }

        _centerTagObservation = null;
        _offsetTagObservation = null;

        double bestPairScore = Double.POSITIVE_INFINITY;
        for (List<Integer> hubPair : hubPairs)
        {
            TurretDirector.TagObservation centerCandidate = observationsById.get(hubPair.get(0));
            TurretDirector.TagObservation offsetCandidate = observationsById.get(hubPair.get(1));
            if (centerCandidate == null || offsetCandidate == null)
            {
                continue;
            }

            double pairScore = (centerCandidate.distanceMeters() + offsetCandidate.distanceMeters()) / 2.0;
            if (pairScore < bestPairScore)
            {
                bestPairScore         = pairScore;
                _centerTagObservation = centerCandidate;
                _offsetTagObservation = offsetCandidate;
            }
        }

        if (_centerTagObservation == null)
        {
            double closestCenterDistance = Double.POSITIVE_INFINITY;
            for (List<Integer> hubPair : hubPairs)
            {
                TurretDirector.TagObservation centerCandidate = observationsById.get(hubPair.get(0));
                if (centerCandidate == null || centerCandidate.distanceMeters() >= closestCenterDistance)
                {
                    continue;
                }

                closestCenterDistance = centerCandidate.distanceMeters();
                _centerTagObservation = centerCandidate;
            }
        }

        _hasTarget = targetData.getTargetStatus() || _centerTagObservation != null;
    }

    private void stopTurret()
    {
        _hasSetpoint              = false;
        _turretSetpoint           = _defaultSetpoint;
        _continuousTurretSetpoint = _continuousRobotAngle;
        _distanceToHub            = Meters.zero();
        _turretMotor.setVoltage(0.0);
    }

    private static Angle clampToTurretLimits(Angle angle)
    {
        return MeasureUtil.clamp(angle, ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE);
    }

    private void syncMotorEncoderToPotentiometer()
    {
        Angle sensorAngle = getCalibratedRobotAngle();
        _robotTurretAngle         = sensorAngle;
        _lastWrappedRobotAngle    = sensorAngle;
        _continuousRobotAngle     = _lastWrappedRobotAngle;
        _continuousTurretSetpoint = _continuousRobotAngle;

        double motorRotations = _continuousRobotAngle.times(ShooterConstants.TURRET_GEAR_RATIO).in(Rotations);
        _turretMotor.setPosition(motorRotations);
    }

    private Angle getCalibratedRobotAngle()
    {
        return clampToTurretLimits(Degrees.of(_turretSensor.get()));
    }

    private void updateContinuousRobotAngle(Angle wrappedAngle)
    {
        Angle delta = Degrees.of(MathUtil.inputModulus(wrappedAngle.minus(_lastWrappedRobotAngle).in(Degrees), -180.0, 180.0));
        _continuousRobotAngle  = _continuousRobotAngle.plus(delta);
        _lastWrappedRobotAngle = wrappedAngle;
    }

    private Angle chooseContinuousSetpoint(Angle requestedWrappedSetpoint)
    {
        requestedWrappedSetpoint = moveSetpointOutOfDeadZone(requestedWrappedSetpoint);
        Angle currentWrapped = wrap(_continuousRobotAngle);
        Angle shortestDelta  = Degrees.of(MathUtil.inputModulus(requestedWrappedSetpoint.minus(currentWrapped).in(Degrees), -180.0, 180.0));
        Angle alternateDelta = shortestDelta.gt(Degrees.zero()) ? shortestDelta.minus(Degrees.of(360.0)) : shortestDelta.plus(Degrees.of(360.0));

        if (!pathCrossesDeadZone(currentWrapped, shortestDelta))
        {
            return _continuousRobotAngle.plus(shortestDelta);
        }

        if (!pathCrossesDeadZone(currentWrapped, alternateDelta))
        {
            return _continuousRobotAngle.plus(alternateDelta);
        }

        return _continuousRobotAngle;
    }

    private Angle moveSetpointOutOfDeadZone(Angle requestedWrappedSetpoint)
    {
        if (ShooterConstants.TURRET_DEAD_ZONE_WIDTH.lte(Degrees.zero()) || !isInDeadZone(requestedWrappedSetpoint))
        {
            return requestedWrappedSetpoint;
        }

        Angle  deadZoneCenter = wrap(ShooterConstants.TURRET_DEAD_ZONE_CENTER);
        Angle  halfWidth      = ShooterConstants.TURRET_DEAD_ZONE_WIDTH.div(2.0);
        Angle  leftEdge       = wrap(deadZoneCenter.minus(halfWidth));
        Angle  rightEdge      = wrap(deadZoneCenter.plus(halfWidth));
        double leftDelta      = Math.abs(MathUtil.inputModulus(requestedWrappedSetpoint.minus(leftEdge).in(Degrees), -180.0, 180.0));
        double rightDelta     = Math.abs(MathUtil.inputModulus(requestedWrappedSetpoint.minus(rightEdge).in(Degrees), -180.0, 180.0));

        return leftDelta <= rightDelta ? leftEdge : rightEdge;
    }

    private boolean pathCrossesDeadZone(Angle startWrapped, Angle delta)
    {
        if (ShooterConstants.TURRET_DEAD_ZONE_WIDTH.lte(Degrees.zero()))
        {
            return false;
        }

        if (delta.abs(Degrees) <= 1e-9)
        {
            return isInDeadZone(startWrapped);
        }

        double zoneCenter   = wrap(ShooterConstants.TURRET_DEAD_ZONE_CENTER).in(Degrees);
        double halfWidth    = ShooterConstants.TURRET_DEAD_ZONE_WIDTH.div(2.0).in(Degrees);
        double endUnwrapped = startWrapped.plus(delta).in(Degrees);

        double startExclusive = delta.gt(Degrees.zero()) ? Math.nextUp(startWrapped.in(Degrees)) : Math.nextDown(startWrapped.in(Degrees));
        double low            = Math.min(startExclusive, endUnwrapped);
        double high           = Math.max(startExclusive, endUnwrapped);

        // Exact interval intersection against periodic dead-zone copies (center+360k),
        // which is robust across +/-180 wrap boundaries.
        int kMin = (int)Math.floor((low - zoneCenter) / 360.0) - 1;
        int kMax = (int)Math.ceil((high - zoneCenter) / 360.0) + 1;
        for (int k = kMin; k <= kMax; k++)
        {
            double centerK      = zoneCenter + (360.0 * k);
            double intervalLow  = centerK - halfWidth;
            double intervalHigh = centerK + halfWidth;
            if (intervalHigh >= low && intervalLow <= high)
            {
                return true;
            }
        }

        return false;
    }

    private boolean isInDeadZone(Angle wrappedAngle)
    {
        double centerDelta = Math.abs(MathUtil.inputModulus(wrappedAngle.minus(ShooterConstants.TURRET_DEAD_ZONE_CENTER).in(Degrees), -180.0, 180.0));
        return centerDelta <= ShooterConstants.TURRET_DEAD_ZONE_WIDTH.div(2.0).in(Degrees);
    }

    private static Angle wrap(Angle angle)
    {
        return Degrees.of(MathUtil.inputModulus(angle.in(Degrees), -180.0, 180.0));
    }

    private static Angle getTurretPotFullRange()
    {
        return ShooterConstants.TURRET_MAX_ANGLE.minus(ShooterConstants.TURRET_MIN_ANGLE).times(GeneralConstants.SENSOR_VOLTAGE.div(ShooterConstants.TURRET_POTENTIOMETER_MAX_VOLTS.minus(ShooterConstants.TURRET_POTENTIOMETER_MIN_VOLTS)));
    }

    private static Angle getTurretPotOffset()
    {
        return ShooterConstants.TURRET_MIN_ANGLE.minus((ShooterConstants.TURRET_POTENTIOMETER_MIN_VOLTS.div(GeneralConstants.SENSOR_VOLTAGE)).times(getTurretPotFullRange())).plus(ShooterConstants.TURRET_POTENTIOMETER_ZERO_OFFSET);
    }
}
