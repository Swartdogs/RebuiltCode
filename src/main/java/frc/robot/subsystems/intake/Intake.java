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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
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
            case Forward -> IntakeConstants.INTAKE_VOLTS;
            case Reverse -> IntakeConstants.REVERSE_VOLTS;
            case Off -> Volts.zero();
        };

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
}
