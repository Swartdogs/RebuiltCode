package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
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
import limelight.results.RawFiducial;

@Logged
public class Turret extends SubsystemBase
{
    public enum TurretState
    {
        Idle, Track, Pass
    }

    private final TalonFX                    _turretMotor;
    private final TalonFXSimState            _turretMotorSim;
    private final DCMotorSim                 _motorSimModel;
    private final AnalogPotentiometer        _turretSensor;
    private final AnalogInputSim             _turretSensorSim;
    private final Limelight                  _limelight;
    private final Supplier<SwerveDriveState> _swerveStateSupplier;
    private final PositionVoltage            _positionRequest = new PositionVoltage(0).withSlot(0);
    private final TurretDirector             _turretDirector  = new TurretDirector();
    private List<Integer>                    _cachedTagFilter;
    private final Angle                      _defaultSetpoint = Degrees.of(ShooterConstants.TURRET_HOME_ANGLE);
    private SwerveDriveState                 _swerveDriveState;
    @Logged
    private Angle                            _fieldTurretAngle;
    @Logged
    private Angle                            _robotTurretAngle;
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
    private TagObservation                   _centerTagObservation;
    private TagObservation                   _leftTagObservation;

    public Turret(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        _turretMotor         = new TalonFX(CANConstants.TURRET_MOTOR);
        _swerveStateSupplier = swerveStateSupplier;
        _swerveDriveState    = new SwerveDriveState();

        _fieldTurretAngle       = Degrees.zero();
        _robotTurretAngle       = Degrees.zero();
        _turretSetpoint         = _defaultSetpoint;
        _hasSetpoint            = false;
        _turretMotorVoltage     = Volts.zero();
        _turretState            = TurretState.Idle;
        _hasTarget              = false;
        _targetHorizontalOffset = Degrees.zero();
        _distanceToHubMeters    = 0.0;
        _cachedTagFilter        = List.of();
        _centerTagObservation   = null;
        _leftTagObservation     = null;

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

        AnalogInput turretSensorInput = new AnalogInput(AIOConstants.TURRET_POTENTIOMETER);
        _turretSensor = new AnalogPotentiometer(turretSensorInput, ShooterConstants.TURRET_MAX_ANGLE - ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MIN_ANGLE);
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
            _turretSensorSim = new AnalogInputSim(turretSensorInput);
            var gearbox = DCMotor.getKrakenX44(1);
            _motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, ShooterConstants.TURRET_GEAR_RATIO), gearbox);
        }
    }

    /**
     * This function executes automatically every 20 milliseconds. For the turret
     * specifically, we need to ensure the turret is behaving according to the rules
     * defined by the state the turret is in. If in <code>Idle</code>, the turret
     * shouldn't rotate at all, even if the robot rotates. If in <code>Shoot</code>
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
        _robotTurretAngle   = Degrees.of(_turretSensor.get());
        _fieldTurretAngle   = _robotTurretAngle.plus(robotHeading);
        _turretMotorVoltage = _turretMotor.getMotorVoltage().getValue();

        updateVisionData();

        TurretAimSolution aimSolution = _turretDirector.getAimSolution(_turretState, _fieldTurretAngle, robotHeading, _targetHorizontalOffset, _hasTarget, _centerTagObservation, _leftTagObservation);

        if (!aimSolution.hasSetpoint())
        {
            stopTurret();
            return;
        }

        _distanceToHubMeters = aimSolution.distanceToHubMeters();
        Angle requestedRobotSetpoint = clampToTurretLimits(aimSolution.fieldSetpoint().minus(robotHeading));
        _turretSetpoint = requestedRobotSetpoint;
        _hasSetpoint    = true;

        double targetMotorRotations = _turretSetpoint.in(Rotations) * ShooterConstants.TURRET_GEAR_RATIO;
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
        _turretSensorSim.setVoltage(RoboRioSim.getUserVoltage5V() * normalized);
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

        return _robotTurretAngle.isNear(_turretSetpoint, Degrees.of(ShooterConstants.TURRET_TOLERANCE));
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
            _leftTagObservation     = null;
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

        int centerTagId = Utilities.isRedAlliance() ? ShooterConstants.RED_CENTER_TAG_ID : ShooterConstants.BLUE_CENTER_TAG_ID;
        int leftTagId   = Utilities.isRedAlliance() ? ShooterConstants.RED_LEFT_TAG_ID : ShooterConstants.BLUE_LEFT_TAG_ID;

        _centerTagObservation = null;
        _leftTagObservation   = null;
        for (RawFiducial fiducial : limelightData.getRawFiducials())
        {
            if (fiducial.id == centerTagId)
            {
                _centerTagObservation = new TagObservation(fiducial.id, Degrees.of(fiducial.txnc), fiducial.distToRobot);
            }
            else if (fiducial.id == leftTagId)
            {
                _leftTagObservation = new TagObservation(fiducial.id, Degrees.of(fiducial.txnc), fiducial.distToRobot);
            }
        }

        _hasTarget = targetData.getTargetStatus() || _centerTagObservation != null;
    }

    private void stopTurret()
    {
        _hasSetpoint         = false;
        _turretSetpoint      = _defaultSetpoint;
        _distanceToHubMeters = 0.0;
        _turretMotor.setVoltage(0.0);
    }

    private static Angle clampToTurretLimits(Angle angle)
    {
        return Degrees.of(MathUtil.clamp(angle.in(Degrees), ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE));
    }

    private void syncMotorEncoderToPotentiometer()
    {
        Angle  sensorAngle    = clampToTurretLimits(Degrees.of(_turretSensor.get()));
        double motorRotations = sensorAngle.in(Rotations) * ShooterConstants.TURRET_GEAR_RATIO;
        _turretMotor.setPosition(motorRotations);
    }

    /**
     * The TurretDirector is responsible for choosing a target for the robot to aim
     * at and communicating the desired turret angle in field-relative space back to
     * the main Turret subsystem.
     */
    private static class TurretDirector
    {
        public TurretAimSolution getAimSolution(TurretState turretState, Angle currentFieldAngle, Angle robotHeading, Angle horizontalOffset, boolean hasTarget, TagObservation centerTagObservation, TagObservation leftTagObservation)
        {
            return switch (turretState)
            {
                case Idle -> TurretAimSolution.none();
                case Track -> getTrackSolution(currentFieldAngle, robotHeading, horizontalOffset, hasTarget, centerTagObservation, leftTagObservation);
                case Pass -> TurretAimSolution.of(ShooterConstants.TURRET_PASS_TARGET, 0.0);
            };
        }

        private TurretAimSolution getTrackSolution(Angle currentFieldAngle, Angle robotHeading, Angle horizontalOffset, boolean hasTarget, TagObservation centerTagObservation, TagObservation leftTagObservation)
        {
            if (centerTagObservation != null && leftTagObservation != null)
            {
                TriangulationResult triangulation = triangulate(centerTagObservation, leftTagObservation);
                if (triangulation.valid())
                {
                    return TurretAimSolution.of(currentFieldAngle.plus(Degrees.of(triangulation.relativeAngleDeg())), triangulation.distanceMeters());
                }
            }

            if (hasTarget)
            {
                double fallbackDistance = centerTagObservation == null ? 0.0 : centerTagObservation.distanceMeters();
                return TurretAimSolution.of(currentFieldAngle.plus(horizontalOffset), fallbackDistance);
            }

            return TurretAimSolution.of(robotHeading.plus(Degrees.of(ShooterConstants.TURRET_HOME_ANGLE)), 0.0);
        }

        /**
         * Solves for robot-to-hub angle and distance using two known triangles.
         * <p>
         * Triangle R-C-L: Robot (R), center tag (C), left tag (L). We know RC and RL
         * from vision and CL from field drawings (TURRET_CL_METERS). Law of sines gives
         * angle at C.
         * <p>
         * Triangle R-C-H: Hub center (H) is TURRET_CH_METERS from C (field drawing).
         * Law of cosines gives RH (distance to hub). Law of sines gives angle at R from
         * R→C to R→H. We add the camera-to-center-tag angle (tx) to get full
         * robot-frame angle to hub.
         */
        private TriangulationResult triangulate(TagObservation centerTagObservation, TagObservation leftTagObservation)
        {
            double rcDistanceMeters = centerTagObservation.distanceMeters();
            double rlDistanceMeters = leftTagObservation.distanceMeters();
            if (rcDistanceMeters <= 0.0 || rlDistanceMeters <= 0.0)
            {
                return TriangulationResult.invalid();
            }

            double angleLrcRad = Math.toRadians(leftTagObservation.horizontalOffset().in(Degrees) - centerTagObservation.horizontalOffset().in(Degrees));
            double sinLcr      = (rlDistanceMeters * Math.sin(angleLrcRad)) / ShooterConstants.TURRET_CL_METERS;
            double angleLcrRad = Math.asin(MathUtil.clamp(sinLcr, -1.0, 1.0));

            double angleLcrPlusNinetyRad = angleLcrRad + Math.PI / 2.0;
            double rhSquared             = (rcDistanceMeters * rcDistanceMeters) + (ShooterConstants.TURRET_CH_METERS * ShooterConstants.TURRET_CH_METERS)
                    - (2.0 * rcDistanceMeters * ShooterConstants.TURRET_CH_METERS * Math.cos(angleLcrPlusNinetyRad));
            if (rhSquared <= 0.0 || !Double.isFinite(rhSquared))
            {
                return TriangulationResult.invalid();
            }

            double rhDistanceMeters = Math.sqrt(rhSquared);
            double sinHrc           = (ShooterConstants.TURRET_CH_METERS * Math.sin(angleLcrPlusNinetyRad)) / rhDistanceMeters;
            double angleHrcDeg      = Math.toDegrees(Math.asin(MathUtil.clamp(sinHrc, -1.0, 1.0)));

            double angleRhDeg = angleHrcDeg + centerTagObservation.horizontalOffset().in(Degrees);
            if (!Double.isFinite(angleRhDeg))
            {
                return TriangulationResult.invalid();
            }

            return TriangulationResult.valid(angleRhDeg, rhDistanceMeters);
        }
    }

    private static class TurretAimSolution
    {
        private final boolean _hasSetpoint;
        private final Angle   _fieldSetpoint;
        private final double  _distanceToHubMeters;

        private TurretAimSolution(boolean hasSetpoint, Angle fieldSetpoint, double distanceToHubMeters)
        {
            _hasSetpoint         = hasSetpoint;
            _fieldSetpoint       = fieldSetpoint;
            _distanceToHubMeters = distanceToHubMeters;
        }

        public static TurretAimSolution none()
        {
            return new TurretAimSolution(false, Degrees.zero(), 0.0);
        }

        public static TurretAimSolution of(Angle fieldSetpoint, double distanceToHubMeters)
        {
            return new TurretAimSolution(true, fieldSetpoint, distanceToHubMeters);
        }

        public boolean hasSetpoint()
        {
            return _hasSetpoint;
        }

        public Angle fieldSetpoint()
        {
            return _fieldSetpoint;
        }

        public double distanceToHubMeters()
        {
            return _distanceToHubMeters;
        }
    }

    private static class TriangulationResult
    {
        private final boolean _valid;
        private final double  _relativeAngleDeg;
        private final double  _distanceMeters;

        private TriangulationResult(boolean valid, double relativeAngleDeg, double distanceMeters)
        {
            _valid            = valid;
            _relativeAngleDeg = relativeAngleDeg;
            _distanceMeters   = distanceMeters;
        }

        public static TriangulationResult invalid()
        {
            return new TriangulationResult(false, 0.0, 0.0);
        }

        public static TriangulationResult valid(double relativeAngleDeg, double distanceMeters)
        {
            return new TriangulationResult(true, relativeAngleDeg, distanceMeters);
        }

        public boolean valid()
        {
            return _valid;
        }

        public double relativeAngleDeg()
        {
            return _relativeAngleDeg;
        }

        public double distanceMeters()
        {
            return _distanceMeters;
        }
    }

    private static class TagObservation
    {
        private final int    _id;
        private final Angle  _horizontalOffset;
        private final double _distanceMeters;

        private TagObservation(int id, Angle horizontalOffset, double distanceMeters)
        {
            _id               = id;
            _horizontalOffset = horizontalOffset;
            _distanceMeters   = distanceMeters;
        }

        public int id()
        {
            return _id;
        }

        public Angle horizontalOffset()
        {
            return _horizontalOffset;
        }

        public double distanceMeters()
        {
            return _distanceMeters;
        }
    }
}
