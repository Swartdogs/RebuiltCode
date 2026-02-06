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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import limelight.Limelight;

@Logged
public class Turret extends SubsystemBase 
{
    public enum TurretState
    {
        Idle, Track, Pass
    }

    private TalonFX _turretMotor; 
    private AnalogPotentiometer _turretSensor; 
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

        _turretSensor = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER); 
        _limelight = new Limelight(ShooterConstants.LIMELIGHT_NAME);
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
