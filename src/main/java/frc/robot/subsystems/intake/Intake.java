package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

@Logged
public class Intake extends SubsystemBase
{
    public enum IntakeState
    {
        Off, Forward, Reverse
    }

    private final SparkFlex _extensionMotor;
    private final SparkFlex _intakeMotor;
    private final SparkFlexSim _extensionMotorSim;
    private final SparkFlexSim _intakeMotorSim;
    private final UsbCamera _camera;
    @Logged
    private IntakeState     _intakeState  = IntakeState.Off;
    @Logged
    private boolean         _extended;
    @Logged
    private Voltage         _intakeMotorVoltage = Volts.of(0.0);
    @Logged
    private Voltage         _extensionMotorVoltage = Volts.of(0.0);
    @Logged
    private Distance        _extensionMotorPosition = Inches.of(0.0);
    @Logged
    private boolean         _retractedLimitSwitchTriggered = false;
    @Logged
    private boolean         _extendedLimitSwitchTriggered = false;
    
    public Intake()
    {
        _intakeMotor = new SparkFlex(Constants.CAN.INTAKE, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.Intake.CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        _intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isReal())
        {
            _camera = CameraServer.startAutomaticCapture(Constants.Intake.CAMERA_NAME, Constants.Intake.CAMERA_DEVICE_INDEX);
            _camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            _camera.setResolution(Constants.Intake.CAMERA_WIDTH, Constants.Intake.CAMERA_HEIGHT);
            _camera.setFPS(Constants.Intake.CAMERA_FPS);
            _intakeMotorSim = null;
            _extensionMotorSim = null;
        }
        else
        {
            _intakeMotorSim = new SparkFlexSim(_intakeMotor, DCMotor.getNeoVortex(1));
            _extensionMotorSim = new SparkFlexSim(_extensionMotor, DCMotor.getNeoVortex(1));
        }
    }

    @Override
    public void periodic()
    {
        _intakeMotorVoltage = Volts.of(_intakeMotor.getAppliedOutput() * _intakeMotor.getBusVoltage());
        _extensionMotorVoltage = Volts.of(_extensionMotor.getAppliedOutput() * _extensionMotor.getBusVoltage());
        _extensionMotorPosition = Inches.of(_extensionMotor.getEncoder().getPosition());
        _retractedLimitSwitchTriggered = _extensionMotor.getReverseLimitSwitch().isPressed();
        _extendedLimitSwitchTriggered = _extensionMotor.getForwardLimitSwitch().isPressed();
    }

    @Override
    public void simulationPeriodic()
    {
        _extensionMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _intakeMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());

        _extensionMotorSim.getForwardLimitSwitchSim().setPressed(_extensionMotorSim.getRelativeEncoderSim().getPosition() >  1000);
        _extensionMotorSim.getReverseLimitSwitchSim().setPressed(_extensionMotorSim.getRelativeEncoderSim().getPosition() <= 0);
    }

    public void setIntakeState(IntakeState state)
    {
        _intakeState = state;

        _intakeMotor.setVoltage(switch(_intakeState)
        {
            case Forward -> Constants.Intake.INTAKE_VOLTS;
            case Reverse -> Constants.Intake.REVERSE_VOLTS;
            default      -> 0.0;
        });
    }

    public IntakeState getIntakeState()
    {
        return _intakeState;
    }

    public void setExtended(boolean isExtended)
    {
        
    }

    public boolean isExtended()
    {
        return _extended;
    }
}
