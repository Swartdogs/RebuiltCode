package frc.robot.subsystems.intake;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

@Logged
public class Intake extends SubsystemBase
{
    public enum IntakeState
    {
        On, Off, Reverse
    }

    private final IntakeIO _io;
    private final IntakeIO.IntakeIOInputs _inputs = new IntakeIO.IntakeIOInputs();
    private double                         _inVolts  = Constants.Intake.INTAKE_VOLTS;
    private double                         _outVolts = Constants.Intake.REVERSE_VOLTS;
    private UsbCamera                      _camera;

    public Intake(IntakeIO io)
    {
        _io = io;
    }

    public static Intake create()
    {
        IntakeIO io = switch (Constants.CURRENT_MODE)
        {
            case REAL -> new IntakeIOSparkFlex();
            case SIM -> new IntakeIOSim();
            default -> new IntakeIO() {};
        };
        return new Intake(io);
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
    }

    public void set(IntakeState state)
    {
        _io.setVolts(switch (state)
        {
            case On -> _inVolts;
            case Reverse -> _outVolts;
            case Off -> 0;
            default -> 0;
        });
    }

    public void initializeCamera() 
    {
        if (_camera != null || Constants.CURRENT_MODE != Constants.Mode.REAL)
        {
            return;
        }

        _camera = CameraServer.startAutomaticCapture(Constants.Intake.CAMERA_NAME, Constants.Intake.CAMERA_DEVICE_INDEX);
        _camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        _camera.setResolution(Constants.Intake.CAMERA_WIDTH, Constants.Intake.CAMERA_HEIGHT);
        _camera.setFPS(Constants.Intake.CAMERA_FPS);
    }

    public void setInSpeed(double speed)
    {
        _inVolts = Constants.General.MOTOR_VOLTAGE * MathUtil.clamp(speed, 0.0, 1.0);
    }

    public void setOutSpeed(double speed)
    {
        _outVolts = -Constants.General.MOTOR_VOLTAGE * MathUtil.clamp(speed, 0.0, 1.0);
    }

    public double getSpeed()
    {
        return _inputs.motorVolts / Constants.General.MOTOR_VOLTAGE;
    }

    public boolean isIntaking()
    {
        return _inputs.motorVolts > 0;
    }
}
