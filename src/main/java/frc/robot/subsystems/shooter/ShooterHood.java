package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;

import frc.robot.Constants;

@Logged
public class ShooterHood
{
    private final WPI_VictorSPX  _hoodMotor;
    private final AnalogInput    _hoodSensor;
    private final AnalogInputSim _hoodSensorSim;
    private final PIDController  _pid;
    @Logged
    private double               _angle         = 0.0;
    private Double               _angleSetpoint = null;
    private double               _simAngle      = 20.0;

    public ShooterHood()
    {
        _hoodMotor  = new WPI_VictorSPX(Constants.CAN.HOOD_MOTOR);
        _hoodSensor = new AnalogInput(Constants.Shooter.HOOD_ANALOG_INPUT);

        if (RobotBase.isReal())
        {
            _hoodMotor.configFactoryDefault();
            _hoodMotor.setNeutralMode(NeutralMode.Brake);
            _hoodMotor.setInverted(false); // TODO: Check direction
            _hoodSensorSim = null;
        }
        else
        {
            _hoodSensorSim = new AnalogInputSim(_hoodSensor);
            _hoodSensorSim.setVoltage(2.5);
        }

        _pid = new PIDController(Constants.Shooter.HOOD_KP, Constants.Shooter.HOOD_KI, Constants.Shooter.HOOD_KD);
        _pid.setTolerance(Constants.Shooter.HOOD_TOLERANCE);
    }

    public void periodic()
    {
        double rawValue = _hoodSensor.getValue();
        _angle = convertRawToAngle(rawValue);

        if (_angleSetpoint != null)
        {
            double output = _pid.calculate(_angle);
            output = MathUtil.clamp(output, -1.0, 1.0);
            _hoodMotor.set(output);
        }
        else
        {
            _hoodMotor.set(0);
        }
    }

    public void simulationPeriodic()
    {
        double motorOutput = _hoodMotor.get();
        double maxSpeed    = 45.0;

        _simAngle += motorOutput * maxSpeed * Constants.General.LOOP_PERIOD_SECS;
        _simAngle  = MathUtil.clamp(_simAngle, Constants.Shooter.HOOD_MIN_ANGLE, Constants.Shooter.HOOD_MAX_ANGLE);

        double simRawValue = convertAngleToRaw(_simAngle);
        _hoodSensorSim.setVoltage(simRawValue * 5.0 / 4096.0);
    }

    private double convertRawToAngle(double rawValue)
    {
        // TODO: Calibrate with actual hardware
        double rawMin = Constants.Shooter.HOOD_RAW_MIN;
        double rawMax = Constants.Shooter.HOOD_RAW_MAX;

        double normalized = (rawValue - rawMin) / (rawMax - rawMin);
        return Constants.Shooter.HOOD_MIN_ANGLE + normalized * (Constants.Shooter.HOOD_MAX_ANGLE - Constants.Shooter.HOOD_MIN_ANGLE);
    }

    private double convertAngleToRaw(double angle)
    {
        double normalized = (angle - Constants.Shooter.HOOD_MIN_ANGLE) / (Constants.Shooter.HOOD_MAX_ANGLE - Constants.Shooter.HOOD_MIN_ANGLE);
        return Constants.Shooter.HOOD_RAW_MIN + normalized * (Constants.Shooter.HOOD_RAW_MAX - Constants.Shooter.HOOD_RAW_MIN);
    }

    public void setAngle(double angleDegrees)
    {
        _angleSetpoint = MathUtil.clamp(angleDegrees, Constants.Shooter.HOOD_MIN_ANGLE, Constants.Shooter.HOOD_MAX_ANGLE);
        _pid.setSetpoint(_angleSetpoint);
    }

    public void stop()
    {
        _angleSetpoint = null;
        _hoodMotor.set(0);
    }

    public boolean atSetpoint()
    {
        if (_angleSetpoint == null) return true;
        return _pid.atSetpoint();
    }

    public double getAngle()
    {
        return _angle;
    }

    public Double getAngleSetpoint()
    {
        return _angleSetpoint;
    }
}
