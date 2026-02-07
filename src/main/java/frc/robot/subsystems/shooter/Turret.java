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
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.revrobotics.jni.CANCommonJNI;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
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

    private TalonFX _turretMotor; 
    private TalonFXSimState _turretSimMotor; 
    private DCMotorSim _motorSimModel; 
    private AnalogPotentiometer _turretSensor;
    private AnalogEncoderSim _turretSimSensor; 
    private Limelight _limelight; 
    @Logged
    private Angle _robotTurretAngle; 
    @Logged
    private Angle _fieldTurretAngle; 
    @Logged
    private Voltage _turretMotorVoltage; 
    @Logged
    private TurretState _turretState; 
    private Supplier<SwerveDriveState> _swerveStateSupplier; 
    private SwerveDriveState _swerveDriveState;
    private PositionVoltage _positionRequest = new PositionVoltage(0).withSlot(0);

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
        if (RobotBase.isReal()) 
        {
            _turretSensor = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER); 
            _limelight = new Limelight(ShooterConstants.LIMELIGHT_NAME);
            _turretSimMotor = null; 
            _motorSimModel = null; 
        }
        else 
        {
            _turretSimMotor = _turretMotor.getSimState();
            _limelight = null; 
            _turretSensor = null; 
            var gearbox = DCMotor.getKrakenX44Foc(1);
            _motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, ShooterConstants.TURRET_GEAR_RATIO), gearbox);
        }
        _robotTurretAngle = Degrees.of(0.0);
        _fieldTurretAngle = Degrees.of(0.0);
        _turretMotorVoltage = Volts.of(0.0); 
        _turretState = TurretState.Idle; 
        _swerveStateSupplier = (swerveStateSupplier == null) ? () -> new SwerveDriveState() : swerveStateSupplier;
    }

    @Override
    public void periodic()
    {
        _robotTurretAngle = _turretMotor.getPosition().getValue(); 
        _swerveDriveState = _swerveStateSupplier.get(); 
        _fieldTurretAngle = _robotTurretAngle.plus(_swerveDriveState.Pose.getRotation().getMeasure()); 
        _turretMotorVoltage = _turretMotor.getMotorVoltage().getValue();
        _limelight.getSettings().withArilTagIdFilter(Utilities.getOurHubTagIds()).save();

        Angle newAngle = switch(_turretState) {
            case Idle -> _robotTurretAngle;

            case Track -> Degrees.of(_limelight.getData().targetData.getHorizontalOffset());

            // angle target in pass:
            // angle = targetAngle - robotAngle 
            case Pass -> ShooterConstants.TURRET_PASS_TARGET.minus(_swerveDriveState.Pose.getRotation().getMeasure());

            default -> Degrees.of(0.0);
        };
        _turretMotor.setControl(_positionRequest.withPosition(newAngle));
    }

    @Override
    public void simulationPeriodic()
    {
        _turretSimMotor.setSupplyVoltage(RoboRioSim.getVInVoltage());

        Voltage motorVoltage = _turretSimMotor.getMotorVoltageMeasure(); 
        _motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        _motorSimModel.update(GeneralConstants.LOOP_PERIOD_SECS);

        _turretSimMotor.setRawRotorPosition(_motorSimModel.getAngularPosition().times(ShooterConstants.TURRET_GEAR_RATIO));
        _turretSimMotor.setRotorVelocity(_motorSimModel.getAngularVelocity().times(ShooterConstants.TURRET_GEAR_RATIO));
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
        return _turretMotorVoltage.in(Volts) < 0.02; // TODO: return true if the motor's voltage is smaller than a specific value 
    }
}
