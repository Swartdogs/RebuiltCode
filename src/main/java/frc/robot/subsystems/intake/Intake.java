package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;

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
        return runOnce(() -> setExtended(true)).andThen(Commands.waitUntil(() -> !waitForExtend || isExtended()));
    }

    public Command retract()
    {
        return retract(false);
    }

    public Command retract(boolean waitForRetract)
    {
        return runOnce(() -> setExtended(false)).andThen(Commands.waitUntil(() -> !waitForRetract || !isExtended()));
    }

    public Command startRollers()
    {
        return startEnd(() -> setIntakeState(IntakeState.Forward), () -> setIntakeState(IntakeState.Off)).onlyIf(this::isExtended);
    }

    public Command reverseRollers()
    {
        return startEnd(() -> setIntakeState(IntakeState.Reverse), () -> setIntakeState(IntakeState.Off)).onlyIf(this::isExtended);
    }

    public Command stopRollers()
    {
        return runOnce(() -> setIntakeState(IntakeState.Off));
    }

    /*************
     * SUBSYSTEM *
     *************/

    private final SparkFlex    _extensionMotor;
    private final SparkFlex    _intakeMotor;
    private final SparkFlexSim _extensionMotorSim;
    private final SparkFlexSim _intakeMotorSim;
    private final UsbCamera    _camera;
    private final DCMotor      _neoVortex;
    @Logged
    private IntakeState        _intakeState                   = IntakeState.Off;
    @Logged
    private boolean            _extended                      = false;
    @Logged
    private Voltage            _intakeMotorVoltage            = Volts.of(0.0);
    @Logged
    private Voltage            _extensionMotorVoltage         = Volts.of(0.0);
    @Logged
    private Distance           _extensionMotorPosition        = Inches.of(0.0);
    @Logged
    private boolean            _retractedLimitSwitchTriggered = false;
    @Logged
    private boolean            _extendedLimitSwitchTriggered  = false;
    private boolean            _targetExtended                = false;

    public Intake()
    {
        _intakeMotor = new SparkFlex(CANConstants.INTAKE, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE);

        var encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(Constants.Intake.EXTENSION_CONVERSION_FACTOR);

        var limitSwitchConfig = new LimitSwitchConfig();
        limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyOpen).forwardLimitSwitchPosition(Constants.Intake.EXTENSION_MAX_POSITION).forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition)
                .reverseLimitSwitchType(Type.kNormallyOpen).reverseLimitSwitchPosition(Constants.Intake.EXTENSION_MIN_POSITION).reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);

        _intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _extensionMotor.configure(config.apply(encoderConfig).apply(limitSwitchConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isReal())
        {
            _camera = CameraServer.startAutomaticCapture(IntakeConstants.CAMERA_NAME, IntakeConstants.CAMERA_DEVICE_INDEX);
            _camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            _camera.setResolution(IntakeConstants.CAMERA_WIDTH, IntakeConstants.CAMERA_HEIGHT);
            _camera.setFPS(IntakeConstants.CAMERA_FPS);
        }
    }

    @Override
    public void periodic()
    {
        _retractedLimitSwitchTriggered = _extensionMotor.getReverseLimitSwitch().isPressed();
        _extendedLimitSwitchTriggered  = _extensionMotor.getForwardLimitSwitch().isPressed();
        _intakeMotorVoltage            = Volts.of(_intakeMotor.getAppliedOutput() * _intakeMotor.getBusVoltage());
        _extensionMotorVoltage         = Volts.of(_extensionMotor.getAppliedOutput() * _extensionMotor.getBusVoltage());
        _extensionMotorPosition        = Inches.of(_extensionMotor.getEncoder().getPosition());

        if (_retractedLimitSwitchTriggered)
        {
            _extended = false;
        }
        else if (_extendedLimitSwitchTriggered)
        {
            _extended = true;
        }

        if (_targetExtended && _extendedLimitSwitchTriggered)
        {
            setExtensionVoltage(0.0);
        }
        else if (!_targetExtended && _retractedLimitSwitchTriggered)
        {
            setExtensionVoltage(0.0);
        }

        if (!_extended && _intakeState != IntakeState.Off)
        {
            setIntakeState(IntakeState.Off);
        }
    }

    @Override
    public void simulationPeriodic()
    {
        if (_extensionMotorSim == null || _intakeMotorSim == null || _neoVortex == null) return;

        _extensionMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _intakeMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());

        double freeSpeedRpm = RadiansPerSecond.of(_neoVortex.freeSpeedRadPerSec).in(RPM);
        _extensionMotorSim.iterate(_extensionMotorSim.getAppliedOutput() * freeSpeedRpm, RoboRioSim.getVInVoltage(), Constants.General.LOOP_PERIOD_SECS);
        _intakeMotorSim.iterate(_intakeMotorSim.getAppliedOutput() * freeSpeedRpm, RoboRioSim.getVInVoltage(), Constants.General.LOOP_PERIOD_SECS);

        double  position       = _extensionMotorSim.getPosition();
        double  output         = _extensionMotorSim.getAppliedOutput();
        boolean forwardPressed = position >= Constants.Intake.EXTENSION_MAX_POSITION;
        boolean reversePressed = position <= Constants.Intake.EXTENSION_MIN_POSITION;

        _extensionMotorSim.getForwardLimitSwitchSim().setPressed(forwardPressed);
        _extensionMotorSim.getReverseLimitSwitchSim().setPressed(reversePressed);

        if ((forwardPressed && output > 0) || (reversePressed && output < 0))
        {
            _extensionMotorSim.setAppliedOutput(0);
        }
    }

    public void setIntakeState(IntakeState state)
    {
        if (state != IntakeState.Off && !isExtended())
        {
            state = IntakeState.Off;
        }

        _intakeState = state;

        var volts = switch (_intakeState)
        {
            case Forward -> IntakeConstants.INTAKE_VOLTS;
            case Reverse -> IntakeConstants.REVERSE_VOLTS;
            case Off -> 0;
        });

        _intakeMotor.setVoltage(volts);

        if (RobotBase.isSimulation())
        {
            _intakeMotorSim.setAppliedOutput(volts / Constants.General.MOTOR_VOLTAGE);
        }
    }

    public IntakeState getIntakeState()
    {
        return _intakeState;
    }

    public void setExtended(boolean extended)
    {
        _targetExtended = extended;

        if (extended && _extendedLimitSwitchTriggered)
        {
            _extended = true;
            setExtensionVoltage(0.0);
            return;
        }

        if (!extended && _retractedLimitSwitchTriggered)
        {
            _extended = false;
            setExtensionVoltage(0.0);
            return;
        }

        setExtensionVoltage(extended ? Constants.Intake.EXTEND_VOLTS : Constants.Intake.RETRACT_VOLTS);
    }

    public boolean isExtended()
    {
        return _extended;
    }

    private void setExtensionVoltage(double volts)
    {
        _extensionMotor.setVoltage(volts);

        if (RobotBase.isSimulation() && _extensionMotorSim != null)
        {
            _extensionMotorSim.setAppliedOutput(volts / Constants.General.MOTOR_VOLTAGE);
        }
    }
}
