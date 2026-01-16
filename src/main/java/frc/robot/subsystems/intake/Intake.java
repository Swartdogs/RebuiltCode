package frc.robot.subsystems.intake;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged; 
import frc.robot.Constants;

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

    public static Intake create(){
        IntakeIO io = switch (Constants.CURRENT_MODE){
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
        _camera = CameraServer.startAutomaticCapture();
        _camera.setResolution(320, 240); 
        _camera.setFPS(15);
    }

    public void setInSpeed(double speed)
    {
        _inVolts = Constants.General.MOTOR_VOLTAGE * speed;
    }

    public void setOutSpeed(double speed)
    {
        _outVolts = -Constants.General.MOTOR_VOLTAGE * speed;
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