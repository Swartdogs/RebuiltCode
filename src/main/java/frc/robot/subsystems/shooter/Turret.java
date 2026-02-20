package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
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
    private final Angle                      _defaultSetpoint = Degrees.of(ShooterConstants.TURRET_HOME_ANGLE);
    private SwerveDriveState                 _swerveDriveState;
    @Logged
    private Angle                            _fieldTurretAngle;
    @Logged
    private Angle                            _robotTurretAngle;
    @Logged
    private double                           _continuousRobotAngleDeg;
    @Logged
    private double                           _continuousTurretSetpointDeg;
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
    private double                           _distanceToHubMeters;
    private TurretDirector.TagObservation    _centerTagObservation;
    private TurretDirector.TagObservation    _offsetTagObservation;
    private double                           _lastWrappedRobotAngleDeg;

    public Turret(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        _turretMotor         = new TalonFX(CANConstants.TURRET_MOTOR);
        _swerveStateSupplier = swerveStateSupplier;
        _swerveDriveState    = new SwerveDriveState();

        _fieldTurretAngle            = Degrees.zero();
        _robotTurretAngle            = Degrees.zero();
        _turretSetpoint              = _defaultSetpoint;
        _hasSetpoint                 = false;
        _turretMotorVoltage          = Volts.zero();
        _turretState                 = TurretState.Idle;
        _hasTarget                   = false;
        _targetHorizontalOffset      = Degrees.zero();
        _distanceToHubMeters         = 0.0;
        _cachedTagFilter             = List.of();
        _centerTagObservation        = null;
        _offsetTagObservation        = null;
        _continuousRobotAngleDeg     = 0.0;
        _continuousTurretSetpointDeg = 0.0;
        _lastWrappedRobotAngleDeg    = 0.0;

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
        _turretSensor      = new AnalogPotentiometer(_turretSensorInput, getTurretPotFullRange(), getTurretPotOffset());
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
            _motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, ShooterConstants.TURRET_GEAR_RATIO), gearbox);
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
        updateContinuousRobotAngle(_robotTurretAngle.in(Degrees));
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

        _distanceToHubMeters = aimSolution.distanceToHubMeters();
        Angle requestedRobotSetpoint = clampToTurretLimits(aimSolution.fieldSetpoint().minus(robotHeading));
        _continuousTurretSetpointDeg = chooseContinuousSetpoint(requestedRobotSetpoint.in(Degrees));
        _turretSetpoint              = Degrees.of(wrapDegrees(_continuousTurretSetpointDeg));
        _hasSetpoint                 = true;

        double targetMotorRotations = Degrees.of(_continuousTurretSetpointDeg).in(Rotations) * ShooterConstants.TURRET_GEAR_RATIO;
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
        _motorSimModel.update(GeneralConstants.LOOP_PERIOD_SECS);

        _turretMotorSim.setRawRotorPosition(_motorSimModel.getAngularPosition().times(ShooterConstants.TURRET_GEAR_RATIO));
        _turretMotorSim.setRotorVelocity(_motorSimModel.getAngularVelocity().times(ShooterConstants.TURRET_GEAR_RATIO));

        double mechanismAngleDeg = _motorSimModel.getAngularPosition().in(Degrees);
        double clampedAngleDeg   = MathUtil.clamp(mechanismAngleDeg, ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE);
        double normalized        = (clampedAngleDeg - ShooterConstants.TURRET_MIN_ANGLE) / (ShooterConstants.TURRET_MAX_ANGLE - ShooterConstants.TURRET_MIN_ANGLE);
        double sensorVoltage     = ShooterConstants.TURRET_POTENTIOMETER_MIN_VOLTS + (normalized * (ShooterConstants.TURRET_POTENTIOMETER_MAX_VOLTS - ShooterConstants.TURRET_POTENTIOMETER_MIN_VOLTS));
        _turretSensorSim.setVoltage(sensorVoltage);
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

        return Math.abs(_continuousRobotAngleDeg - _continuousTurretSetpointDeg) <= ShooterConstants.TURRET_TOLERANCE;
    }

    public boolean hasTarget()
    {
        return _hasTarget;
    }

    public double getDistanceToTarget()
    {
        if (_distanceToHubMeters > 0.0)
        {
            return _distanceToHubMeters;
        }

        if (_limelight == null || !_hasTarget)
        {
            return 0.0;
        }

        var targetPose = _limelight.getData().targetData.getCameraToTarget();
        return targetPose.getTranslation().toTranslation2d().getNorm();
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
        _hasSetpoint                 = false;
        _turretSetpoint              = _defaultSetpoint;
        _continuousTurretSetpointDeg = _continuousRobotAngleDeg;
        _distanceToHubMeters         = 0.0;
        _turretMotor.setVoltage(0.0);
    }

    private static Angle clampToTurretLimits(Angle angle)
    {
        return Degrees.of(MathUtil.clamp(angle.in(Degrees), ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE));
    }

    private void syncMotorEncoderToPotentiometer()
    {
        Angle sensorAngle = getCalibratedRobotAngle();
        _robotTurretAngle            = sensorAngle;
        _lastWrappedRobotAngleDeg    = sensorAngle.in(Degrees);
        _continuousRobotAngleDeg     = _lastWrappedRobotAngleDeg;
        _continuousTurretSetpointDeg = _continuousRobotAngleDeg;

        double motorRotations = Degrees.of(_continuousRobotAngleDeg).in(Rotations) * ShooterConstants.TURRET_GEAR_RATIO;
        _turretMotor.setPosition(motorRotations);
    }

    private Angle getCalibratedRobotAngle()
    {
        return clampToTurretLimits(Degrees.of(_turretSensor.get()));
    }

    private void updateContinuousRobotAngle(double wrappedAngleDeg)
    {
        double delta = MathUtil.inputModulus(wrappedAngleDeg - _lastWrappedRobotAngleDeg, -180.0, 180.0);
        _continuousRobotAngleDeg  += delta;
        _lastWrappedRobotAngleDeg  = wrappedAngleDeg;
    }

    private double chooseContinuousSetpoint(double requestedWrappedSetpointDeg)
    {
        requestedWrappedSetpointDeg = moveSetpointOutOfDeadZone(requestedWrappedSetpointDeg);
        double currentWrappedDeg = wrapDegrees(_continuousRobotAngleDeg);
        double shortestDelta     = MathUtil.inputModulus(requestedWrappedSetpointDeg - currentWrappedDeg, -180.0, 180.0);
        double alternateDelta    = shortestDelta > 0.0 ? shortestDelta - 360.0 : shortestDelta + 360.0;

        if (!pathCrossesDeadZone(currentWrappedDeg, shortestDelta))
        {
            return _continuousRobotAngleDeg + shortestDelta;
        }

        if (!pathCrossesDeadZone(currentWrappedDeg, alternateDelta))
        {
            return _continuousRobotAngleDeg + alternateDelta;
        }

        return _continuousRobotAngleDeg;
    }

    private double moveSetpointOutOfDeadZone(double requestedWrappedSetpointDeg)
    {
        if (ShooterConstants.TURRET_DEAD_ZONE_WIDTH <= 0.0 || !isInDeadZone(requestedWrappedSetpointDeg))
        {
            return requestedWrappedSetpointDeg;
        }

        double deadZoneCenter = wrapDegrees(ShooterConstants.TURRET_DEAD_ZONE_CENTER);
        double halfWidth      = ShooterConstants.TURRET_DEAD_ZONE_WIDTH / 2.0;
        double leftEdge       = wrapDegrees(deadZoneCenter - halfWidth);
        double rightEdge      = wrapDegrees(deadZoneCenter + halfWidth);
        double leftDelta      = Math.abs(MathUtil.inputModulus(requestedWrappedSetpointDeg - leftEdge, -180.0, 180.0));
        double rightDelta     = Math.abs(MathUtil.inputModulus(requestedWrappedSetpointDeg - rightEdge, -180.0, 180.0));

        return leftDelta <= rightDelta ? leftEdge : rightEdge;
    }

    private boolean pathCrossesDeadZone(double startWrappedDeg, double deltaDeg)
    {
        if (ShooterConstants.TURRET_DEAD_ZONE_WIDTH <= 0.0)
        {
            return false;
        }

        if (Math.abs(deltaDeg) <= 1e-9)
        {
            return isInDeadZone(startWrappedDeg);
        }

        double zoneCenter   = wrapDegrees(ShooterConstants.TURRET_DEAD_ZONE_CENTER);
        double halfWidth    = ShooterConstants.TURRET_DEAD_ZONE_WIDTH / 2.0;
        double endUnwrapped = startWrappedDeg + deltaDeg;

        double startExclusive = deltaDeg > 0.0 ? Math.nextUp(startWrappedDeg) : Math.nextDown(startWrappedDeg);
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

    private boolean isInDeadZone(double wrappedAngleDeg)
    {
        double centerDelta = Math.abs(MathUtil.inputModulus(wrappedAngleDeg - ShooterConstants.TURRET_DEAD_ZONE_CENTER, -180.0, 180.0));
        return centerDelta <= ShooterConstants.TURRET_DEAD_ZONE_WIDTH / 2.0;
    }

    private static double wrapDegrees(double angleDeg)
    {
        return MathUtil.inputModulus(angleDeg, -180.0, 180.0);
    }

    private static double getTurretPotFullRange()
    {
        return ((ShooterConstants.TURRET_MAX_ANGLE - ShooterConstants.TURRET_MIN_ANGLE) * GeneralConstants.SENSOR_VOLTAGE) / (ShooterConstants.TURRET_POTENTIOMETER_MAX_VOLTS - ShooterConstants.TURRET_POTENTIOMETER_MIN_VOLTS);
    }

    private static double getTurretPotOffset()
    {
        double fullRange = getTurretPotFullRange();
        return ShooterConstants.TURRET_MIN_ANGLE - ((ShooterConstants.TURRET_POTENTIOMETER_MIN_VOLTS / GeneralConstants.SENSOR_VOLTAGE) * fullRange) + ShooterConstants.TURRET_POTENTIOMETER_ZERO_OFFSET_DEG;
    }
}
