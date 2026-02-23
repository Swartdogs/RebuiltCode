package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorHook;
import frc.robot.TestHook;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ExtensionMotor;

@Logged
public class Intake extends ExtensionMotor
{
    private static final String kSettingsPrefix = "Dashboard/Dashboard Settings/";

    public enum IntakeState
    {
        Off, Forward, Reverse
    }

    /************
     * COMMANDS *
     ************/

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
    private final SparkFlex    _intakeMotor;
    private final SparkFlexSim _intakeMotorSim;
    private final UsbCamera    _camera;
    private final DCMotor      _neoVortex;
    @Logged
    private IntakeState        _intakeState        = IntakeState.Off;
    @Logged
    private Voltage            _intakeMotorVoltage = Volts.of(0.0);

    public Intake()
    {
        super(CANConstants.INTAKE_EXTEND, IntakeConstants.EXTEND_VOLTS, IntakeConstants.RETRACT_VOLTS, IntakeConstants.EXTENSION_CONVERSION_FACTOR);
        initializeTuningPreferences();

        _intakeMotor = new SparkFlex(CANConstants.INTAKE, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit((int)IntakeConstants.CURRENT_LIMIT.in(Amps)).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        _intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isReal())
        {
            _camera = CameraServer.startAutomaticCapture(IntakeConstants.CAMERA_NAME, IntakeConstants.CAMERA_DEVICE_INDEX);
            _camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            _camera.setResolution(IntakeConstants.CAMERA_WIDTH, IntakeConstants.CAMERA_HEIGHT);
            _camera.setFPS(IntakeConstants.CAMERA_FPS);
            _neoVortex      = null;
            _intakeMotorSim = null;
        }
        else
        {
            _camera         = null;
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
        if (state != IntakeState.Off && !isExtended())
        {
            state = IntakeState.Off;
        }

        _intakeState = state;

        var volts = switch (_intakeState)
        {
            case Forward -> getForwardVoltage();
            case Reverse -> getReverseVoltage();
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

    private static void initPreference(String key, double value)
    {
        Preferences.initDouble(kSettingsPrefix + key, value);
    }

    private static void initializeTuningPreferences()
    {
        initPreference("Intake Forward Voltage", IntakeConstants.INTAKE_VOLTS.in(Volts));
        initPreference("Intake Reverse Voltage", IntakeConstants.REVERSE_VOLTS.in(Volts));
        initPreference("Intake Extension Max Position", IntakeConstants.EXTENSION_MAX_POSITION.in(Inches));
        initPreference("Intake Extend Voltage", IntakeConstants.EXTEND_VOLTS.in(Volts));
        initPreference("Intake Retract Voltage", IntakeConstants.RETRACT_VOLTS.in(Volts));
    }

    private static Voltage getForwardVoltage()
    {
        return Volts.of(Preferences.getDouble(kSettingsPrefix + "Intake Forward Voltage", IntakeConstants.INTAKE_VOLTS.in(Volts)));
    }

    private static Voltage getReverseVoltage()
    {
        return Volts.of(Preferences.getDouble(kSettingsPrefix + "Intake Reverse Voltage", IntakeConstants.REVERSE_VOLTS.in(Volts)));
    }
}
