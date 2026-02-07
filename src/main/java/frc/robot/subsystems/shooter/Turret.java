package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
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
    private AnalogPotentiometer        _turretSensor;
    private AnalogInput                _turretSensorInput;
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
    private Supplier<SwerveDriveState> _swerveStateSupplier;
    private SwerveDriveState           _swerveDriveState;
    private PositionVoltage            _positionRequest = new PositionVoltage(0).withSlot(0);
    @Logged
    private boolean _hasTarget;

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
        _turretSensor = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER, ShooterConstants.TURRET_MAX_ANGLE - ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_TOLERANCE);
        _turretSensorInput = new AnalogInput(AIOConstants.TURRET_POTENTIOMETER); 

        if (RobotBase.isReal())
        {
            _limelight = new Limelight(ShooterConstants.LIMELIGHT_NAME);
            _turretSimMotor = null;
            _motorSimModel = null;
            _turretSimSensor = null;
        }
        else
        {
            _turretSimMotor = _turretMotor.getSimState();
            _limelight = null;
            _turretSimSensor = new AnalogInputSim(_turretSensorInput);
            var gearbox = DCMotor.getKrakenX44Foc(1);
            _motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, ShooterConstants.TURRET_GEAR_RATIO), gearbox);
        }
        _robotTurretAngle = Degrees.of(0.0);
        _fieldTurretAngle = Degrees.of(0.0);
        _turretMotorVoltage = Volts.of(0.0);
        _turretState = TurretState.Idle;
        _swerveStateSupplier = (swerveStateSupplier == null) ? () -> new SwerveDriveState() : swerveStateSupplier;
        _hasTarget = false; 
    }

    @Override
    public void periodic()
    {
        _robotTurretAngle   = _turretMotor.getPosition().getValue();
        _swerveDriveState   = _swerveStateSupplier.get();
        _fieldTurretAngle   = _robotTurretAngle.plus(_swerveDriveState.Pose.getRotation().getMeasure());
        _turretMotorVoltage = _turretMotor.getMotorVoltage().getValue();

        if (_limelight != null)
        {
            _limelight.getSettings().withArilTagIdFilter(Utilities.getOurHubTagIds()).save();
        }

        _turretSetpoint = switch (_turretState)
        {
            case Idle -> _robotTurretAngle;

            case Track -> Degrees.of(_limelight.getData().targetData.getHorizontalOffset()).plus(_robotTurretAngle);

            // angle target in pass:
            // angle = targetAngle - robotAngle
            case Pass -> ShooterConstants.TURRET_PASS_TARGET.minus(_swerveDriveState.Pose.getRotation().getMeasure());

            default -> Degrees.of(0.0);
        };

        _turretSetpoint = Degrees.of(MathUtil.clamp(_turretSetpoint.in(Degrees), ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE));
        double targetMotorRotation = _turretSetpoint.in(Degrees) * ShooterConstants.TURRET_GEAR_RATIO;
        _turretMotor.setControl(_positionRequest.withPosition(targetMotorRotation));

        _hasTarget = _limelight.getData().targetData.getTargetStatus(); 
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

        double motorPosition = _motorSimModel.getAngularPosition().in(Degrees) * ShooterConstants.TURRET_GEAR_RATIO;
        motorPosition = Math.max(ShooterConstants.TURRET_MIN_ANGLE, Math.min(ShooterConstants.TURRET_MAX_ANGLE, motorPosition));
        motorPosition = (motorPosition - ShooterConstants.TURRET_MIN_ANGLE) / ShooterConstants.TURRET_MAX_ANGLE;
        _turretSimSensor.setVoltage(motorPosition * RoboRioSim.getUserVoltage5V());
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
        if (_turretState == TurretState.Idle || _turretSetpoint == null) return true; 
        return Math.abs((_robotTurretAngle.minus(_turretSetpoint)).in(Degrees)) <= ShooterConstants.TURRET_TOLERANCE; 
    }
}
