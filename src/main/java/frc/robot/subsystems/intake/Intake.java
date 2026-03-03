package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.MotorHook;
import frc.robot.TestHook;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ExtensionMotor;

@Logged
public class Intake extends ExtensionMotor
{
    public enum IntakeState
    {
        Off, Forward, Reverse
    }

    /************
     * COMMANDS *
     ************/

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

    public Command startRollersReverse()
    {
        return runOnce(() -> setIntakeState(IntakeState.Reverse));
    }

    public Command stopRollers()
    {
        return runOnce(() -> setIntakeState(IntakeState.Off));
    }

    public Command jiggle()
    {
        // @formatter:off
        return
            runOnce(() -> setIntakeState(IntakeState.Forward))
            .andThen
            (
                Commands.repeatingSequence
                (
                    getExtendCmd(),
                    Commands.waitSeconds(0.4),
                    super.getRetractCmd(),
                    Commands.waitUntil(this::isRetracted)
                )
            )
            .finallyDo(() -> {
                setIntakeState(IntakeState.Off);
                extend(false);
            })
            .onlyIf(this::isRetracted);
        // @formatter:on
    }

    @Override
    public Command getRetractCmd()
    {
        // @formatter:off
        return
            Commands.sequence
            (
                startRollersForward(),
                super.getRetractCmd(),
                Commands.waitUntil(this::isRetracted)
            ).finallyDo(() -> setIntakeState(IntakeState.Off));
        // @formatter:on
    }

    /*************
     * SUBSYSTEM *
     *************/
    private final SparkFlex    _intakeMotor;
    private final SparkFlexSim _intakeMotorSim;
    private final DCMotor      _neoVortex;
    @Logged
    private IntakeState        _intakeState        = IntakeState.Off;
    @Logged
    private Voltage            _intakeMotorVoltage = Volts.of(0.0);

    public Intake()
    {
        super(CANConstants.INTAKE_EXTEND, IntakeConstants.EXTEND_VOLTS, IntakeConstants.RETRACT_VOLTS, IntakeConstants.EXTENSION_CONVERSION_FACTOR);

        _intakeMotor = new SparkFlex(CANConstants.INTAKE, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit((int)IntakeConstants.ROLLER_CURRENT_LIMIT.in(Amps)).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        _intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isReal())
        {
            _neoVortex      = null;
            _intakeMotorSim = null;
        }
        else
        {
            _neoVortex      = DCMotor.getNeoVortex(1);
            _intakeMotorSim = new SparkFlexSim(_intakeMotor, _neoVortex);
        }
    }

    @Override
    public void periodic()
    {
        super.periodic();

        _intakeMotorVoltage = Volts.of(_intakeMotor.getAppliedOutput() * _intakeMotor.getBusVoltage());
    }

    @Override
    public void simulationPeriodic()
    {
        super.simulationPeriodic();

        _intakeMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _intakeMotorSim.iterate(RadiansPerSecond.of(_neoVortex.freeSpeedRadPerSec).times(Value.of(_intakeMotorSim.getAppliedOutput())).in(RPM), RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD.in(Seconds));
    }

    public void setIntakeState(IntakeState state)
    {
        _intakeState = state;

        Voltage volts = switch (_intakeState)
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
