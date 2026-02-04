package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
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
    public enum RollerState
    {
        Off, Forward, Reverse
    }

    public enum ExtendState
    {
        Extended, Retracted, InTransit
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
        return runOnce(() -> setExtendState(ExtendState.Extended)).andThen(Commands.waitUntil(() -> !waitForExtend || (getExtendState() == ExtendState.Extended)));
    }

    public Command retract()
    {
        return retract(false);
    }

    public Command retract(boolean waitForRetract)
    {
        return runOnce(() -> setExtendState(ExtendState.Retracted)).andThen(Commands.waitUntil(() -> !waitForRetract || (getExtendState() == ExtendState.Retracted)));
    }

    public Command startRollers()
    {
        return runOnce(() -> setRollerState(RollerState.Forward)).andThen(Commands.idle(this)).finallyDo(() -> setRollerState(RollerState.Off)).onlyIf(() -> getExtendState() == ExtendState.Extended);
    }

    public Command reverseRollers()
    {
        return runOnce(() -> setRollerState(RollerState.Reverse)).andThen(Commands.idle(this)).finallyDo(() -> setRollerState(RollerState.Off)).onlyIf(() -> getExtendState() == ExtendState.Extended);
    }

    public Command stopRollers()
    {
        return runOnce(() -> setRollerState(RollerState.Off));
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
    private final DCMotorSim   _extensionMotorSimGearbox;
    @Logged
    private RollerState        _rollerState                   = RollerState.Off;
    @Logged
    private ExtendState        _extendState                   = ExtendState.Retracted;
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

    public Intake()
    {
        _intakeMotor    = new SparkFlex(Constants.CAN.INTAKE, MotorType.kBrushless);
        _extensionMotor = new SparkFlex(Constants.CAN.INTAKE_EXTENSION, MotorType.kBrushless);

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
            _camera.setResolution(Constants.Intake.CAMERA_WIDTH, Constants.Intake.CAMERA_HEIGHT);
            _camera.setFPS(Constants.Intake.CAMERA_FPS);
            _intakeMotorSim           = null;
            _extensionMotorSim        = null;
            _neoVortex                = null;
            _extensionMotorSimGearbox = null;
        }
        else
        {
            _camera                   = null;
            _neoVortex                = DCMotor.getNeoVortex(1);
            _extensionMotorSimGearbox = null;
            _intakeMotorSim           = new SparkFlexSim(_intakeMotor, _neoVortex);
            _extensionMotorSim        = new SparkFlexSim(_extensionMotor, _neoVortex);
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
            _extendState = ExtendState.Retracted;
        }
        else if (_extendedLimitSwitchTriggered)
        {
            _extendState = ExtendState.Extended;
        }
        else
        {
            _extendState = ExtendState.InTransit;
        }
    }

    @Override
    public void simulationPeriodic()
    {
        _extensionMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _intakeMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());

        _extensionMotorSim.iterate(_extensionMotorSim.getAppliedOutput() * RadiansPerSecond.of(_neoVortex.freeSpeedRadPerSec).in(RPM), RoboRioSim.getVInVoltage(), Constants.General.LOOP_PERIOD_SECS);

        double position       = _extensionMotorSim.getPosition();
        double output         = _extensionMotorSim.getAppliedOutput();
        boolean forwardPressed = position >= Constants.Intake.EXTENSION_MAX_POSITION;
        boolean reversePressed = position <= Constants.Intake.EXTENSION_MIN_POSITION;

        _extensionMotorSim.getForwardLimitSwitchSim().setPressed(forwardPressed);
        _extensionMotorSim.getReverseLimitSwitchSim().setPressed(reversePressed);

        if ((forwardPressed && output > 0) || (reversePressed && output < 0))
        {
            _extensionMotorSim.setAppliedOutput(0);
        }
    }

    public void setRollerState(RollerState state)
    {
        _rollerState = state;

        var volts = switch (_rollerState)
        {
            case Forward -> Constants.Intake.INTAKE_VOLTS;
            case Reverse -> Constants.Intake.REVERSE_VOLTS;
            default -> 0.0;
        };

        _intakeMotor.setVoltage(volts);

        if (RobotBase.isSimulation())
        {
            _intakeMotorSim.setAppliedOutput(volts / Constants.General.MOTOR_VOLTAGE);
        }
    }

    public RollerState getIntakeState()
    {
        return _rollerState;
    }

    public void setExtendState(ExtendState state)
    {
        if (state == ExtendState.InTransit) return;

        var volts = switch (state)
        {
            case Extended -> Constants.Intake.EXTEND_VOLTS;
            case Retracted -> Constants.Intake.RETRACT_VOLTS;
            default -> 0.0;
        };

        _extensionMotor.setVoltage(volts);

        if (RobotBase.isSimulation())
        {
            _extensionMotorSim.setAppliedOutput(volts / Constants.General.MOTOR_VOLTAGE);
        }
    }

    public ExtendState getExtendState()
    {
        return _extendState;
    }
}
