package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ExtensionMotor;
import frc.robot.subsystems.ExtensionMotor.ExtensionState;

@Logged
public class Intake extends SubsystemBase
{
    public enum IntakeState
    {
        Off, Forward, Reverse
    }

    /************
     * COMMANDS *
     ************/

    public Command extend()
    {
        return extend(false);
    }

    public Command extend(boolean waitForExtend)
    {
        return runOnce(() -> setExtended(true)).andThen(Commands.waitUntil(() -> !waitForExtend || _extensionMotor.isExtended()));
    }

    public Command retract()
    {
        return retract(false);
    }

    public Command retract(boolean waitForRetract)
    {
        return runOnce(() -> setExtended(false)).andThen(Commands.waitUntil(() -> !waitForRetract || !_extensionMotor.isExtended()));
    }

    public Command startRollers()
    {
        return startEnd(() -> setIntakeState(IntakeState.Forward), () -> setIntakeState(IntakeState.Off)).onlyIf(_extensionMotor::isExtended);
    }

    public Command reverseRollers()
    {
        return startEnd(() -> setIntakeState(IntakeState.Reverse), () -> setIntakeState(IntakeState.Off)).onlyIf(_extensionMotor::isExtended);
    }

    public Command stopRollers()
    {
        return runOnce(() -> setIntakeState(IntakeState.Off));
    }

    /*************
     * SUBSYSTEM *
     *************/

    private final ExtensionMotor _extensionMotor;
    private final SparkFlex      _intakeMotor;
    private final UsbCamera      _camera;
    private final DCMotor        _neoVortex;
    @Logged
    private IntakeState          _intakeState                   = IntakeState.Off;
    @Logged
    private Voltage              _intakeMotorVoltage            = Volts.of(0.0);
    @Logged
    private Voltage              _extensionMotorVoltage         = Volts.of(0.0);
    @Logged
    private Distance             _extensionMotorPosition        = Inches.of(0.0);
    @Logged
    private boolean              _retractedLimitSwitchTriggered = false;
    @Logged
    private boolean              _extendedLimitSwitchTriggered  = false;

    public Intake()
    {
        _intakeMotor    = new SparkFlex(CANConstants.INTAKE, MotorType.kBrushless);
        _extensionMotor = new ExtensionMotor(CANConstants.INTAKE_EXTEND, IntakeConstants.EXTEND_OUTPUT, IntakeConstants.RETRACT_VOLTS, IntakeConstants.EXTENSION_MIN_POSITION, IntakeConstants.EXTENSION_MAX_POSITION);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE);

        var encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(IntakeConstants.EXTENSION_CONVERSION_FACTOR);

        var limitSwitchConfig = new LimitSwitchConfig();
        limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyOpen).forwardLimitSwitchPosition(IntakeConstants.EXTENSION_MAX_POSITION.in(Inches)).forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition)
                .reverseLimitSwitchType(Type.kNormallyOpen).reverseLimitSwitchPosition(IntakeConstants.EXTENSION_MIN_POSITION.in(Inches)).reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);

        _intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _camera = CameraServer.startAutomaticCapture(IntakeConstants.CAMERA_NAME, IntakeConstants.CAMERA_DEVICE_INDEX);
        _camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        _camera.setResolution(IntakeConstants.CAMERA_WIDTH, IntakeConstants.CAMERA_HEIGHT);
        _camera.setFPS(IntakeConstants.CAMERA_FPS);
        _neoVortex = null;
    }

    @Override
    public void periodic()
    {
        _retractedLimitSwitchTriggered = _extensionMotor.isRetracted();
        _extendedLimitSwitchTriggered  = _extensionMotor.isExtended();
        _intakeMotorVoltage            = Volts.of(_intakeMotor.getAppliedOutput() * _intakeMotor.getBusVoltage());
        _extensionMotorVoltage         = _extensionMotor.getMotorVoltage();
        _extensionMotorPosition        = _extensionMotor.getMotorPosition();

        if (!_extensionMotor.isExtended() && _intakeState != IntakeState.Off)
        {
            setIntakeState(IntakeState.Off);
        }
    }

    public void setIntakeState(IntakeState state)
    {
        if (state != IntakeState.Off && !_extensionMotor.isExtended())
        {
            state = IntakeState.Off;
        }

        _intakeState = state;

        Voltage volts = switch (_intakeState)
        {
            case Forward -> IntakeConstants.INTAKE_VOLTS;
            case Reverse -> IntakeConstants.REVERSE_VOLTS;
            default -> Volts.zero();
        };

        _intakeMotor.setVoltage(volts);
    }

    public IntakeState getIntakeState()
    {
        return _intakeState;
    }

    public void setExtended(boolean extended)
    {
        _extensionMotor.setExtensionState(extended ? ExtensionState.Extending : ExtensionState.Retracting);
    }
}
