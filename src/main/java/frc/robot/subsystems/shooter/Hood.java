package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Robot;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

@Logged
public class Hood extends SubsystemBase
{
    public enum HoodPosition
    {
        Shoot(ShooterConstants.HOOD_SHOOT_ANGLE), Pass(ShooterConstants.HOOD_PASS_ANGLE), Undefined(null);

        private final Angle _targetAngle;

        private HoodPosition(Angle angle)
        {
            _targetAngle = angle;
        }

        private Angle getTargetAngle()
        {
            return _targetAngle;
        }
    }

    private final WPI_VictorSPX       _hoodMotor;
    private final AnalogPotentiometer _hoodSensor;
    private final PIDController       _pidController;
    private double                    _simAngle;
    @Logged
    private double                    _simMotorCmd;
    private final AnalogInputSim      _hoodSensorSim;
    @Logged
    private Voltage                   _hoodMotorVoltage;
    @Logged
    private Angle                     _hoodAngle;
    @Logged
    private HoodPosition              _hoodPosition;
    @Logged
    private Angle                     _hoodSetpoint;
    @Logged
    private boolean                   _hasSetpoint;

    public Hood()
    {
        AnalogInput hoodSensorInput;

        _hoodMotor        = new WPI_VictorSPX(CANConstants.HOOD_MOTOR);
        hoodSensorInput   = new AnalogInput(AIOConstants.HOOD_POTENTIOMETER);
        _hoodSensor       = new AnalogPotentiometer(hoodSensorInput, ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MIN_ANGLE);
        _hoodMotorVoltage = Volts.zero();
        _hoodAngle        = Degrees.zero();
        _hoodPosition     = HoodPosition.Undefined;
        _pidController    = new PIDController(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD);
        _pidController.setTolerance(ShooterConstants.HOOD_TOLERANCE.in(Degrees));
        _hoodSetpoint = Degrees.zero();
        _hasSetpoint  = false;
        _simAngle     = ShooterConstants.HOOD_PASS_ANGLE.in(Degrees);

        _simMotorCmd = 0.0;

        if (RobotBase.isReal())
        {
            _hoodMotor.configFactoryDefault();
            _hoodMotor.setNeutralMode(NeutralMode.Brake);
            _hoodMotor.setInverted(false); // TODO: Check direction
            _hoodSensorSim = null;

        }
        else
        {
            _hoodSensorSim = new AnalogInputSim(hoodSensorInput);
            updateSimSensorVoltage();
        }
    }

    @Override
    public void periodic()
    {
        _hoodMotorVoltage = Volts.of(_hoodMotor.getMotorOutputVoltage());
        _hoodAngle        = Degrees.of(_hoodSensor.get());

        if (_hasSetpoint)
        {
            double voltageOutput = _pidController.calculate(_hoodAngle.in(Degrees), _hoodSetpoint.in(Degrees));
            setHoodMotorVoltage(Volts.of(voltageOutput));
        }
        else
        {
            setHoodMotorVoltage(Volts.zero());
        }
    }

    @Override
    public void simulationPeriodic()
    {
        if (_hoodSensorSim == null) return;
        _simAngle += _simMotorCmd * ShooterConstants.HOOD_SIM_MAX_SPEED * GeneralConstants.LOOP_PERIOD_SECS;
        _simAngle  = MathUtil.clamp(_simAngle, ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);

        updateSimSensorVoltage();
    }

    public void stop()
    {
        setHoodPosition(HoodPosition.Undefined);
    }

    public void setHoodMotorVoltage(Voltage voltage)
    {
        double clampedVoltage = MathUtil.clamp(voltage.in(Volts), -GeneralConstants.MOTOR_VOLTAGE, GeneralConstants.MOTOR_VOLTAGE);
        _hoodMotor.setVoltage(clampedVoltage);

        if (RobotBase.isSimulation())
        {
            _simMotorCmd = clampedVoltage;
        }
    }

    public void setHoodPosition(HoodPosition hoodPosition)
    {
        // if (hoodPosition == null) hoodPosition = HoodPosition.Undefined;
        _hoodPosition = hoodPosition;
        Angle targetAngle = hoodPosition.getTargetAngle();
        if (targetAngle == null)
        {
            _hasSetpoint = false;
            setHoodMotorVoltage(Volts.zero());
            return;
        }
        double degreesClamped = MathUtil.clamp(targetAngle.in(Degrees), ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);
        _hoodSetpoint = Degrees.of(degreesClamped);
        _hasSetpoint  = true;
        _pidController.setSetpoint(degreesClamped);

    }

    public boolean atSetpoint()
    {
        if (!_hasSetpoint) return true;
        return _hoodAngle.isNear(_hoodSetpoint, ShooterConstants.HOOD_TOLERANCE);
    }

    private void updateSimSensorVoltage()
    {
        double normalized = (_simAngle - ShooterConstants.HOOD_MIN_ANGLE) / (ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE);
        normalized = MathUtil.clamp(normalized, 0.0, 1.0);

        _hoodSensorSim.setVoltage(RoboRioSim.getUserVoltage5V() * normalized);
    }

    public HoodPosition getHoodPosition()
    {
        return _hoodPosition;
    }

    public Command getSetPositionCmd(HoodPosition hoodPosition)
    {
        return runOnce(() -> setHoodPosition(hoodPosition));
    }

    public Command getStopCmd()
    {
        return runOnce(this::stop);
    }
}
