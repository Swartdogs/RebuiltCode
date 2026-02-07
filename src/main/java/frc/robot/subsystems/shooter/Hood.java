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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GeneralConstants;
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

        private Angle _targetAngle;

        private HoodPosition(Angle angle)
        {
            _targetAngle = angle;
        }

        public boolean atPosition(Angle angle)
        {
            if (_targetAngle == null) return true;
            double difference = angle.minus(_targetAngle).abs(Degrees);
            return difference < ShooterConstants.HOOD_TOLERANCE.in(Degrees);
        }
    }

    private final WPI_VictorSPX       _hoodMotor;
    private final AnalogPotentiometer _hoodSensor;
    private final PIDController       _pidController;
    private double                    _simAngle; // TODO: Check if behavior can be done with just _hoodAngle
    private final AnalogInputSim      _hoodSensorSim;
    private final AnalogInput         _hoodSensorInput;
    @Logged
    private Voltage                   _hoodMotorVoltage;
    @Logged
    private Angle                     _hoodAngle;
    @Logged
    private HoodPosition              _hoodPosition;
    @Logged
    private Angle                     _hoodSetpoint;

    public Hood()
    {
        _hoodMotor        = new WPI_VictorSPX(CANConstants.HOOD_MOTOR);
        _hoodSensorInput  = new AnalogInput(AIOConstants.HOOD_POTENTIOMETER);
        _hoodSensor       = new AnalogPotentiometer(_hoodSensorInput, ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MIN_ANGLE);
        _hoodMotorVoltage = Volts.of(0.0);
        _hoodAngle        = Degrees.of(0.0);
        _hoodPosition     = HoodPosition.Undefined;
        _pidController    = new PIDController(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD);
        _pidController.setTolerance(ShooterConstants.HOOD_TOLERANCE.in(Degrees));
        _hoodSetpoint = null;
        _simAngle     = ShooterConstants.HOOD_PASS_ANGLE.in(Degrees);

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
        _hoodMotorVoltage = Volts.of(_hoodMotor.getMotorOutputVoltage());
        _hoodAngle        = Degrees.of(_hoodSensor.get());

        if (_hoodSetpoint != null)
        {
            double voltageOutput = _pidController.calculate(_hoodAngle.in(Degrees), _hoodSetpoint.in(Degrees));
            _hoodMotor.setVoltage(MathUtil.clamp(voltageOutput, -GeneralConstants.MOTOR_VOLTAGE, GeneralConstants.MOTOR_VOLTAGE));
        }
        else
        {
            _hoodMotor.setVoltage(0.0);
        }

        for (HoodPosition position : HoodPosition.values())
        {
            if (position.atPosition(_hoodAngle))
            {
                _hoodPosition = position;
                break;
            }
        }
    }

    public void simulationPeriodic()
    {
        double motorOutput = _hoodMotor.get();
        double maxSpeed    = ShooterConstants.HOOD_SIM_MAX_SPEED;

        _simAngle += motorOutput * maxSpeed * GeneralConstants.LOOP_PERIOD_SECS;
        _simAngle  = MathUtil.clamp(_simAngle, ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);

        updateSimSensorVoltage();
    }

    public void stop()
    {
        setHoodPosition(HoodPosition.Undefined);
        _hoodMotor.setVoltage(0.0);
    }

    public void setHoodPosition(HoodPosition hoodPosition)
    {
        if (hoodPosition == null) hoodPosition = HoodPosition.Undefined;
        _hoodSetpoint = switch (hoodPosition)
        {
            case Shoot -> ShooterConstants.HOOD_SHOOT_ANGLE;
            case Pass -> ShooterConstants.HOOD_PASS_ANGLE;
            default -> null;
        };
        _pidController.setSetpoint(_hoodSetpoint.in(Degrees));
    }

    private void updateSimSensorVoltage() // TODO: There is very likely a better way to do this.
    {
        double normalized = (_simAngle - ShooterConstants.HOOD_MIN_ANGLE) / (ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE);
        normalized = MathUtil.clamp(normalized, 0.0, 1.0);

        _hoodSensorSim.setVoltage(RoboRioSim.getUserVoltage5V() * normalized);
    }

    public HoodPosition getHoodPosition()
    {
        return _hoodPosition;
    }
}
