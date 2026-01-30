package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

@Logged
public class Intake extends SubsystemBase
{
    public enum IntakeState
    {
        Forward, Off, Reverse
    }

    private Command setCommand(IntakeState state)
    {
        return startEnd(() -> set(state), () -> set(IntakeState.Off));
    }

    public Command getForwardCmd()
    {
        return setCommand(IntakeState.Forward);
    }

    public Command getReverseCmd()
    {
        return setCommand(IntakeState.Reverse);
    }

    public Command getOffCmd()
    {
        return runOnce(() -> set(IntakeState.Off));
    }

    private final SparkFlex _intakeMotor;
    private UsbCamera       _camera;
    @Logged
    private Voltage         _motorVoltage = Volts.of(0.0);
    @Logged
    private IntakeState     _intakeState  = IntakeState.Off;

    public Intake()
    {
        _intakeMotor = new SparkFlex(Constants.CAN.INTAKE, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit((int)Constants.Intake.CURRENT_LIMIT.in(Amps)).voltageCompensation(Constants.General.MOTOR_VOLTAGE.in(Volts));

        _intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isReal())
        {
            _camera = CameraServer.startAutomaticCapture(Constants.Intake.CAMERA_NAME, Constants.Intake.CAMERA_DEVICE_INDEX);
            _camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            _camera.setResolution(Constants.Intake.CAMERA_WIDTH, Constants.Intake.CAMERA_HEIGHT);
            _camera.setFPS(Constants.Intake.CAMERA_FPS);
        }
    }

    @Override
    public void periodic()
    {
        _motorVoltage = Volts.of(_intakeMotor.getBusVoltage()).times(Value.of(_intakeMotor.getAppliedOutput()));
    }

    public void set(IntakeState state)
    {
        _intakeMotor.setVoltage(switch (state)
        {
            case Forward -> Constants.Intake.INTAKE_VOLTS;
            case Reverse -> Constants.Intake.REVERSE_VOLTS;
            case Off     -> Volts.of(0.0);
        });

        _intakeState = state;
    }
}
