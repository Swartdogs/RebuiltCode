package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class ShooterHood
{
    private final WPI_VictorSPX       _hoodMotor;
    private final AnalogInput         _hoodSensorInput;
    private final AnalogPotentiometer _hoodSensor;
    private final AnalogInputSim      _hoodSensorSim;
    private final PIDController       _pid;
    @Logged
    private double                    _angle         = 0.0;
    private Double                    _angleSetpoint = null;
    private double                    _simAngle      = 20.0;

    public ShooterHood()
    {
        _hoodMotor       = new WPI_VictorSPX(CANConstants.HOOD_MOTOR);
        _hoodSensorInput = new AnalogInput(AIOConstants.HOOD_POTENTIOMETER);
        _hoodSensor      = new AnalogPotentiometer(_hoodSensorInput, ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MIN_ANGLE);

        _pid = new PIDController(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD);
        _pid.setTolerance(ShooterConstants.HOOD_TOLERANCE);

        if (RobotBase.isReal())
        {
            _hoodMotor.configFactoryDefault();
            _hoodMotor.setNeutralMode(NeutralMode.Brake);
            _hoodMotor.setInverted(false); // TODO: Check direction
            _hoodSensorSim = null;
        }
        else
        {
            _hoodSensorSim = new AnalogInputSim(_hoodSensorInput);
            updateSimSensorVoltage();
        }
    }

    public void periodic()
    {
        _angle = _hoodSensor.get();

        double speed = 0.0;
        if (_angleSetpoint != null)
        {
            speed = _pid.calculate(_angle);
        }
        _hoodMotor.set(speed);
    }

    public void simulationPeriodic()
    {
        double motorOutput = _hoodMotor.get();
        double maxSpeed    = ShooterConstants.HOOD_SIM_MAX_SPEED;

        _simAngle += motorOutput * maxSpeed * GeneralConstants.LOOP_PERIOD_SECS;
        _simAngle  = MathUtil.clamp(_simAngle, ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);

        updateSimSensorVoltage();
    }

    private void updateSimSensorVoltage()
    {
        double normalized = (_simAngle - ShooterConstants.HOOD_MIN_ANGLE) / (ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE);
        normalized = MathUtil.clamp(normalized, 0.0, 1.0);

        _hoodSensorSim.setVoltage(RoboRioSim.getUserVoltage5V() * normalized);
    }

    public void setAngle(double angleDegrees)
    {
        _angleSetpoint = MathUtil.clamp(angleDegrees, ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);
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
