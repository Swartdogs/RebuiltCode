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
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
        // @formatter:off

        Off(Volts.zero()),
        Forward(IntakeConstants.ROLLER_FORWARD_VOLTS),
        Reverse(IntakeConstants.ROLLER_REVERSE_VOLTS),
        SysId(Volts.zero());

        // @formatter:on

        public Voltage voltage;

        private RollerState(Voltage voltage)
        {
            this.voltage = voltage;
        }
    }

    public enum ExtendState
    {
        // @formatter:off

        Homing(Inches.zero()),
        Extended(IntakeConstants.EXTENSION_MAX_POSITION),
        Retracted(IntakeConstants.EXTENSION_MIN_POSITION),
        SysId(Inches.zero());

        // @formatter:on

        public Distance distance;

        private ExtendState(Distance distance)
        {
            this.distance = distance;
        }
    }

    private final SparkFlex                 _rollerMotor;
    private final SparkFlex                 _extendMotor;
    private final SparkFlexSim              _rollerMotorSim;
    private final SparkFlexSim              _extendMotorSim;
    private final SparkClosedLoopController _extendMotorPid;
    private final DCMotor                   _rollerMotorModel;
    private final DCMotor                   _extendMotorModel;
    @Logged
    private RollerState                     _rollerState;
    @Logged
    private ExtendState                     _extendState;
    @Logged
    private Distance                        _extendDistance;
    @Logged
    private Voltage                         _rollerMotorVoltage;
    @Logged
    private Voltage                         _extendMotorVoltage;
    @Logged
    private Current                         _rollerMotorCurrent;
    @Logged
    private Current                         _extendMotorCurrent;
    @Logged
    private Voltage                         _sysIdRollerMotorVoltage;
    @Logged
    private Voltage                         _sysIdExtendMotorVoltage;
    @NotLogged
    private Debouncer                       _homingDebouncer;
    @NotLogged
    private Timer                           _homingTimer;

    public Intake()
    {
        _rollerMotor = new SparkFlex(CANConstants.INTAKE_EXTEND, MotorType.kBrushless);
        _extendMotor = new SparkFlex(CANConstants.INTAKE_ROLLER, MotorType.kBrushless);

        _extendMotorPid = _extendMotor.getClosedLoopController();

        // @formatter:off

        var rollerMotorConfig = new SparkFlexConfig();

        rollerMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int)IntakeConstants.ROLLER_CURRENT_LIMIT_ACTIVE.in(Amps))
            .voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        _rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var extendMotorConfig = new SparkFlexConfig();

        extendMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int)IntakeConstants.EXTENSION_CURRENT_LIMIT.in(Amps))
            .voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        extendMotorConfig.encoder
            .positionConversionFactor(IntakeConstants.EXTENSION_CONVERSION_FACTOR.in(Inches.per(Rotation)));

        extendMotorConfig.closedLoop
            .p(0)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .allowedClosedLoopError(IntakeConstants.EXTENSION_PID_TOLERANCE.in(Inches), ClosedLoopSlot.kSlot0);

        extendMotorConfig.closedLoop.feedForward
            .kG(0)
            .kS(0)
            .kV(0)
            .kA(0);

        _extendMotor.configure(extendMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // @formatter:on

        if (RobotBase.isReal())
        {
            _rollerMotorModel = null;
            _extendMotorModel = null;
            _rollerMotorSim   = null;
            _extendMotorSim   = null;
        }
        else
        {
            _rollerMotorModel = DCMotor.getNeoVortex(1);
            _extendMotorModel = DCMotor.getNeoVortex(1);
            _rollerMotorSim   = new SparkFlexSim(_rollerMotor, _rollerMotorModel);
            _extendMotorSim   = new SparkFlexSim(_extendMotor, _extendMotorModel);
        }

        _rollerState    = RollerState.Off;
        _extendState    = ExtendState.Homing;
        _extendDistance = Inches.zero();

        _rollerMotorVoltage = Volts.zero();
        _rollerMotorCurrent = Amps.zero();

        _extendMotorVoltage = Volts.zero();
        _extendMotorCurrent = Amps.zero();

        _sysIdRollerMotorVoltage = Volts.zero();
        _sysIdExtendMotorVoltage = Volts.zero();

        _homingDebouncer = new Debouncer(IntakeConstants.EXTENSION_HOMING_DEBOUNCE_TIME.in(Seconds), DebounceType.kRising);
        _homingTimer     = new Timer();
    }

    public void setRollerState(RollerState desiredState)
    {
        _rollerState = desiredState;
    }

    public void setExtendState(ExtendState desiredState)
    {
        // Only allow going to another state if we've finished homing
        if (_extendState != ExtendState.Homing)
        {
            _extendState = desiredState;
        }
    }

    public boolean isRetracted()
    {
        return _extendState == ExtendState.Retracted && _extendMotorPid.isAtSetpoint();
    }

    public boolean isExtended()
    {
        return _extendState == ExtendState.Extended && _extendMotorPid.isAtSetpoint();
    }

    private void setRollerCurrentLimit(Current limit)
    {
        _rollerMotor.configure(new SparkFlexConfig().smartCurrentLimit((int)limit.in(Amps)), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic()
    {
        // Sensor measurements
        _extendDistance = Inches.of(_extendMotor.getEncoder().getPosition());

        _rollerMotorVoltage = Volts.of(_rollerMotor.getAppliedOutput() * _rollerMotor.getBusVoltage());
        _rollerMotorCurrent = Amps.of(_rollerMotor.getOutputCurrent());

        _extendMotorVoltage = Volts.of(_extendMotor.getAppliedOutput() * _extendMotor.getBusVoltage());
        _extendMotorCurrent = Amps.of(_extendMotor.getOutputCurrent());

        // Control loops

        // Clear the SysId roller motor voltage if SysId is not active
        if (_rollerState != RollerState.SysId)
        {
            _sysIdRollerMotorVoltage = Volts.zero();
        }

        // Determine the roller motor voltage based on roller state
        var rollerMotorVolts = switch (_rollerState)
        {
            case Off, Forward, Reverse -> _rollerState.voltage;
            case SysId -> _sysIdRollerMotorVoltage;
            default -> Volts.zero();
        };

        // Set the roller output voltage
        _rollerMotor.setVoltage(rollerMotorVolts);

        // Set the roller applied output in simulation
        if (RobotBase.isSimulation())
        {
            _rollerMotorSim.setAppliedOutput(rollerMotorVolts.div(GeneralConstants.MOTOR_VOLTAGE).in(Value));
        }

        // Clear the SysId extend motor voltage if SysId is not active
        if (_extendState != ExtendState.SysId)
        {
            _sysIdExtendMotorVoltage = Volts.zero();
        }

        // Determine the extend motor behavior based on extend state
        switch (_extendState)
        {
            case Homing:
                // Only perform homing operations if the robot is enabled
                if (DriverStation.isEnabled())
                {
                    // If we're enabled and homing but the timer isn't running, we need to
                    // start the timer and clear the debouncer (by making a new instance)
                    if (!_homingTimer.isRunning())
                    {
                        _homingTimer.restart();
                        _homingDebouncer = new Debouncer(IntakeConstants.EXTENSION_HOMING_DEBOUNCE_TIME.in(Seconds), DebounceType.kRising);
                    }

                    // The timer will be running here and the debouncer has been created.
                    //
                    // Check to make sure that one of the following is true:
                    // 1. The motor current has been above its threshold for the debounce time
                    // 2. The homing algorithm has been running for its full time limit
                    //
                    // If either of the above are true, turn off the motor, reset the encoder,
                    // set our new state, and stop the timer.
                    if (_homingDebouncer.calculate(_extendMotorCurrent.gt(IntakeConstants.EXTENSION_HOMING_CURRENT_THRESHOLD)) || _homingTimer.hasElapsed(IntakeConstants.EXTENSION_HOMING_TIME_LIMIT))
                    {
                        _extendMotor.setVoltage(0);
                        _extendMotor.getEncoder().setPosition(ExtendState.Retracted.distance.in(Inches));
                        _extendState = ExtendState.Retracted;
                        _homingTimer.stop();
                    }
                    // The robot is enabled, but we haven't exceeded the homing current threshold
                    // for long enough and haven't been running the homing algorithm for too long.
                    else
                    {
                        _extendMotor.setVoltage(IntakeConstants.EXTENSION_HOMING_VOLTAGE);
                    }
                }
                // The robot is disabled. Stop the timer and shut off the motor. We'll restart
                // the timer once we're enabled again.
                else
                {
                    _homingTimer.stop();
                    _extendMotor.setVoltage(Volts.zero());
                }
                break;

            case Extended:
            case Retracted:
                _extendMotorPid.setSetpoint(_extendState.distance.in(Inches), ControlType.kPosition);
                break;

            case SysId:
                _extendMotor.setVoltage(_sysIdExtendMotorVoltage);
                break;

            default:
                _extendMotor.setVoltage(Volts.zero());
                break;
        }
    }

    @Override
    public void simulationPeriodic()
    {
        _extendMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _extendMotorSim.iterate(_extendMotorSim.getAppliedOutput() * RadiansPerSecond.of(_extendMotorModel.freeSpeedRadPerSec).in(RPM), RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD.in(Seconds));

        _rollerMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _rollerMotorSim.iterate(RadiansPerSecond.of(_rollerMotorModel.freeSpeedRadPerSec).times(Value.of(_rollerMotorSim.getAppliedOutput())).in(RPM), RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD.in(Seconds));
    }

    public Command runRollersForward()
    {
        return startEnd(() -> setRollerState(RollerState.Forward), () -> setRollerState(RollerState.Off));
    }

    public Command startRollersForward()
    {
        return runOnce(() -> setRollerState(RollerState.Forward));
    }

    public Command runRollersReverse()
    {
        return startEnd(() -> setRollerState(RollerState.Reverse), () -> setRollerState(RollerState.Off));
    }

    public Command getRetractCmd()
    {
        return Commands.sequence(runOnce(() -> setRollerCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT_ACTIVE)), startRollersForward(), runOnce(() -> setExtendState(ExtendState.Retracted)), Commands.waitUntil(this::isRetracted))
                .withTimeout(2.0).finallyDo(() ->
                {
                    setRollerState(RollerState.Off);
                    setRollerCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT_EXTENDED);
                });
    }

    public Command getExtendCmd()
    {
        return runOnce(() -> setExtendState(ExtendState.Extended));
    }

    public Command jiggle()
    {
        return runOnce(() ->
        {
            setRollerState(RollerState.Forward);
            setRollerCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT_ACTIVE);
            setExtendState(ExtendState.Extended);
        }).andThen(
                Commands.waitUntil(this::isExtended),
                Commands.repeatingSequence(
                        runOnce(() -> setExtendState(ExtendState.Extended)), Commands.waitSeconds(IntakeConstants.JIGGLE_MOVE_TIMEOUT.in(Seconds)), runOnce(() -> setExtendState(ExtendState.Retracted)),
                        Commands.waitSeconds(IntakeConstants.JIGGLE_MOVE_TIMEOUT.in(Seconds))
                )
        ).finallyDo(() ->
        {
            setRollerState(RollerState.Off);
            setRollerCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT_EXTENDED);
            setExtendState(ExtendState.Retracted);
        });
    }
}
