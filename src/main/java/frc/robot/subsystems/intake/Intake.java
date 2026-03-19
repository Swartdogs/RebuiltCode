package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.test.MotorHook;
import frc.robot.subsystems.test.TestHook;

@Logged
public class Intake extends SubsystemBase
{
    private enum RollerCurrentLimitMode
    {
        AutoByExtension, ForceActive
    }

    public enum IntakeState
    {
        Off, Forward, Reverse
    }

    public Command runRollersForward()
    {
        return startEnd(() -> setIntakeState(IntakeState.Forward), () -> setIntakeState(IntakeState.Off));
    }

    public Command startRollersForward()
    {
        return runOnce(() -> setIntakeState(IntakeState.Forward));
    }

    public Command runRollersReverse()
    {
        return startEnd(() -> setIntakeState(IntakeState.Reverse), () -> setIntakeState(IntakeState.Off));
    }

    public Command jiggle()
    {
        Distance extensionRange       = IntakeConstants.EXTENSION_MAX_POSITION.minus(IntakeConstants.EXTENSION_MIN_POSITION);
        double   extensionRangeInches = extensionRange.in(Inches);

        return runOnce(() ->
        {
            setIntakeVoltage(GeneralConstants.MOTOR_VOLTAGE);
            setRollerCurrentLimitMode(RollerCurrentLimitMode.ForceActive);
            _jiggleRetractFraction     = IntakeConstants.JIGGLE_RETRACT_FRACTION;
            _jiggleRetractTravelInches = extensionRangeInches * _jiggleRetractFraction;
        }).andThen(
                Commands.sequence(
                        runOnce(() -> setExtensionVoltage(IntakeConstants.JIGGLE_EXTEND_VOLTS)), Commands.waitUntil(this::isExtended).withTimeout(IntakeConstants.JIGGLE_MOVE_TIMEOUT.in(Seconds)),
                        runOnce(() -> setExtensionVoltage(Volts.zero())), Commands.waitSeconds(IntakeConstants.JIGGLE_PAUSE_TIME.in(Seconds)), Commands.repeatingSequence(runOnce(() ->
                        {
                            _jiggleLegStartInches = getMotorPosition().in(Inches);
                            _jiggleRetractTravelInches = extensionRangeInches * _jiggleRetractFraction;
                        }), runOnce(() -> setExtensionVoltage(IntakeConstants.JIGGLE_RETRACT_VOLTS)),
                                Commands.waitUntil(() -> isRetracted() || Math.abs(getMotorPosition().in(Inches) - _jiggleLegStartInches) >= _jiggleRetractTravelInches).withTimeout(IntakeConstants.JIGGLE_MOVE_TIMEOUT.in(Seconds)),
                                runOnce(() -> setExtensionVoltage(Volts.zero())), Commands.waitSeconds(IntakeConstants.JIGGLE_PAUSE_TIME.in(Seconds)), runOnce(() -> setExtensionVoltage(IntakeConstants.JIGGLE_EXTEND_VOLTS)),
                                Commands.waitUntil(this::isExtended).withTimeout(IntakeConstants.JIGGLE_MOVE_TIMEOUT.in(Seconds)), runOnce(() -> setExtensionVoltage(Volts.zero())),
                                Commands.waitSeconds(IntakeConstants.JIGGLE_PAUSE_TIME.in(Seconds)), runOnce(() -> _jiggleRetractFraction = Math.min(1.0, _jiggleRetractFraction + IntakeConstants.JIGGLE_RETRACT_STEP_FRACTION))
                        )
                )
        ).finallyDo(() ->
        {
            setIntakeState(IntakeState.Off);
            setRollerCurrentLimitMode(RollerCurrentLimitMode.AutoByExtension);
            setExtensionVoltage(Volts.zero());
        });
    }

    public Command getRetractCmd()
    {
        return Commands.sequence(runOnce(() -> setRollerCurrentLimitMode(RollerCurrentLimitMode.ForceActive)), startRollersForward(), setExtensionCmd(false), Commands.waitUntil(this::isRetracted)).finallyDo(() ->
        {
            setIntakeState(IntakeState.Off);
            setRollerCurrentLimitMode(RollerCurrentLimitMode.AutoByExtension);
        });
    }

    @NotLogged
    public Command getExtendCmd()
    {
        return setExtensionCmd(true);
    }

    private Command setExtensionCmd(boolean finalState)
    {
        return runOnce(() -> extend(finalState));
    }

    private final SparkFlex        _intakeMotor;
    private final SparkFlex        _extendMotor;
    private final SparkLimitSwitch _outLimitSwitch;
    private final SparkLimitSwitch _inLimitSwitch;
    private final SparkFlexSim     _intakeMotorSim;
    private final SparkFlexSim     _extensionMotorSim;
    private final DCMotor          _neoVortex;
    private final DCMotor          _extensionMotorModel;
    private final Alert            _limitSwitchAlert;
    @NotLogged
    private final SparkFlexConfig  _rollerBaseConfig;
    @NotLogged
    private double                 _jiggleLegStartInches        = 0.0;
    @NotLogged
    private double                 _jiggleRetractTravelInches   = 0.0;
    @NotLogged
    private double                 _jiggleRetractFraction       = IntakeConstants.JIGGLE_RETRACT_FRACTION;
    @NotLogged
    private RollerCurrentLimitMode _rollerCurrentLimitMode      = RollerCurrentLimitMode.AutoByExtension;
    @NotLogged
    private int                    _lastAppliedCurrentLimitAmps = Integer.MIN_VALUE;
    @Logged
    private IntakeState            _intakeState                 = IntakeState.Off;
    @Logged
    private Voltage                _intakeMotorVoltage          = Volts.of(0.0);
    @Logged
    private Distance               _currentExtension            = Inches.zero();
    @Logged
    private Voltage                _motorVoltage                = Volts.zero();
    @Logged
    private boolean                _outSwitchTriggered          = false;
    @Logged
    private boolean                _inSwitchTriggered           = false;

    public Intake()
    {
        _extendMotor    = new SparkFlex(CANConstants.INTAKE_EXTEND, MotorType.kBrushless);
        _outLimitSwitch = _extendMotor.getForwardLimitSwitch();
        _inLimitSwitch  = _extendMotor.getReverseLimitSwitch();

        var extensionConfig = new SparkFlexConfig();
        extensionConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit((int)IntakeConstants.EXTENSION_CURRENT_LIMIT.in(Amps)).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        var encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(IntakeConstants.EXTENSION_CONVERSION_FACTOR.in(Inches.per(Rotation)));

        _extendMotor.setVoltage(Volts.zero());

        var limitSwitchConfig = new LimitSwitchConfig();
        limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyOpen).forwardLimitSwitchPosition(IntakeConstants.EXTENSION_MAX_POSITION.in(Inches)).forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
                .reverseLimitSwitchType(Type.kNormallyOpen).reverseLimitSwitchPosition(IntakeConstants.EXTENSION_MIN_POSITION.in(Inches)).reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);

        _extendMotor.configure(extensionConfig.apply(encoderConfig).apply(limitSwitchConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _limitSwitchAlert = new Alert("Both in and out limit switches are triggered for motor CAN ID " + CANConstants.INTAKE_EXTEND + ".", Alert.AlertType.kWarning);

        _intakeMotor = new SparkFlex(CANConstants.INTAKE, MotorType.kBrushless);

        _rollerBaseConfig = new SparkFlexConfig();
        _rollerBaseConfig.inverted(false).idleMode(IdleMode.kBrake).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));
        applyRollerCurrentLimit((int)IntakeConstants.ROLLER_CURRENT_LIMIT_ACTIVE.in(Amps));

        if (RobotBase.isReal())
        {
            _extensionMotorModel = null;
            _extensionMotorSim   = null;
            _neoVortex           = null;
            _intakeMotorSim      = null;
        }
        else
        {
            _extensionMotorModel = DCMotor.getNeoVortex(1);
            _extensionMotorSim   = new SparkFlexSim(_extendMotor, _extensionMotorModel);
            _neoVortex           = DCMotor.getNeoVortex(1);
            _intakeMotorSim      = new SparkFlexSim(_intakeMotor, _neoVortex);
        }
    }

    @Override
    public void periodic()
    {
        _outSwitchTriggered = _outLimitSwitch.isPressed();
        _inSwitchTriggered  = _inLimitSwitch.isPressed();
        _limitSwitchAlert.set(_outSwitchTriggered && _inSwitchTriggered);

        _currentExtension   = Inches.of(_extendMotor.getEncoder().getPosition());
        _motorVoltage       = Volts.of(_extendMotor.getAppliedOutput() * _extendMotor.getBusVoltage());
        _intakeMotorVoltage = Volts.of(_intakeMotor.getAppliedOutput() * _intakeMotor.getBusVoltage());

        applyDesiredRollerCurrentLimit();
    }

    @Override
    public void simulationPeriodic()
    {
        _extensionMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _extensionMotorSim.iterate(_extensionMotorSim.getAppliedOutput() * RadiansPerSecond.of(_extensionMotorModel.freeSpeedRadPerSec).in(RPM), RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD.in(Seconds));

        Distance      position       = Inches.of(_extensionMotorSim.getPosition());
        Dimensionless output         = Value.of(_extensionMotorSim.getAppliedOutput());
        boolean       forwardPressed = position.gte(SimulationConstants.EXTENDED_DISTANCE);
        boolean       reversePressed = position.lte(SimulationConstants.RETRACTED_DISTANCE);

        _extensionMotorSim.getForwardLimitSwitchSim().setPressed(forwardPressed);
        _extensionMotorSim.getReverseLimitSwitchSim().setPressed(reversePressed);

        if ((forwardPressed && output.gt(Value.zero())) || (reversePressed && output.lt(Value.zero())))
        {
            _extensionMotorSim.setAppliedOutput(0);
        }

        _intakeMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _intakeMotorSim.iterate(RadiansPerSecond.of(_neoVortex.freeSpeedRadPerSec).times(Value.of(_intakeMotorSim.getAppliedOutput())).in(RPM), RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD.in(Seconds));
    }

    public void extend(boolean finalState)
    {
        setExtensionVoltage(finalState ? IntakeConstants.EXTEND_VOLTS : IntakeConstants.RETRACT_VOLTS);
    }

    public Voltage getMotorVoltage()
    {
        return Volts.of(_extendMotor.getAppliedOutput() * _extendMotor.getBusVoltage());
    }

    public Distance getMotorPosition()
    {
        return _currentExtension;
    }

    public boolean isExtended()
    {
        return _outSwitchTriggered;
    }

    public boolean isRetracted()
    {
        return _inSwitchTriggered;
    }

    public void setIntakeState(IntakeState state)
    {
        _intakeState = state;

        var volts = switch (_intakeState)
        {
            case Forward -> IntakeConstants.INTAKE_VOLTS;
            case Reverse -> IntakeConstants.REVERSE_VOLTS;
            case Off -> Volts.zero();
        };

        setIntakeVoltage(volts);
    }

    private void setIntakeVoltage(Voltage volts)
    {
        _intakeMotor.setVoltage(volts);

        if (RobotBase.isSimulation())
        {
            _intakeMotorSim.setAppliedOutput(volts.div(GeneralConstants.MOTOR_VOLTAGE).in(Value));
        }
    }

    private void setExtensionVoltage(Voltage volts)
    {
        _extendMotor.setVoltage(volts);

        if (RobotBase.isSimulation())
        {
            _extensionMotorSim.setAppliedOutput(volts.div(GeneralConstants.MOTOR_VOLTAGE).in(Value));
        }
    }

    private void setRollerCurrentLimitMode(RollerCurrentLimitMode mode)
    {
        _rollerCurrentLimitMode = mode;
        applyDesiredRollerCurrentLimit();
    }

    private void applyDesiredRollerCurrentLimit()
    {
        int desiredAmps = switch (_rollerCurrentLimitMode)
        {
            case ForceActive -> (int)IntakeConstants.ROLLER_CURRENT_LIMIT_ACTIVE.in(Amps);
            case AutoByExtension -> isExtended() ? (int)IntakeConstants.ROLLER_CURRENT_LIMIT_EXTENDED.in(Amps) : (int)IntakeConstants.ROLLER_CURRENT_LIMIT_ACTIVE.in(Amps);
        };

        applyRollerCurrentLimit(desiredAmps);
    }

    private void applyRollerCurrentLimit(int amps)
    {
        if (amps == _lastAppliedCurrentLimitAmps)
        {
            return;
        }

        _intakeMotor.configure(_rollerBaseConfig.smartCurrentLimit(amps), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        _lastAppliedCurrentLimitAmps = amps;
    }

    public IntakeState getIntakeState()
    {
        return _intakeState;
    }

    private class IntakeHook extends MotorHook
    {
        @Override
        public void stop()
        {
            _intakeMotor.stopMotor();
        }

        @Override
        public void setRate(double rate)
        {
            setIntakeVoltage(GeneralConstants.MOTOR_VOLTAGE.times(rate * _polarity));
        }
    }

    public TestHook getHook()
    {
        return new IntakeHook();
    }
}
