package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Robot;
import frc.robot.Constants.DIOConstants;
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
    private final DutyCycleEncoder _hoodSensor;
    private final DutyCycleEncoderSim      _hoodSensorSim;
    private final PIDController       _pidController;
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
        _hoodMotor        = new WPI_VictorSPX(CANConstants.HOOD_MOTOR);
        _hoodSensor       = new DutyCycleEncoder(DIOConstants.HOOD_POTENTIOMETER, ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MIN_ANGLE);
        _pidController    = new PIDController(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD);
        _hoodMotorVoltage = Volts.zero();
        _hoodAngle        = Degrees.zero();
        _hoodPosition     = HoodPosition.Undefined;
        _hoodSetpoint = Degrees.zero();
        _hasSetpoint  = false;
   
        _pidController.setTolerance(ShooterConstants.HOOD_TOLERANCE.in(Degrees));
        
        if (RobotBase.isReal())
        {
            _hoodMotor.configFactoryDefault();
            _hoodMotor.setNeutralMode(NeutralMode.Brake);
            _hoodMotor.setInverted(false); // TODO: Check direction
            _hoodSensorSim = null;

        }
        else
        {
            _hoodSensorSim = new DutyCycleEncoderSim(_hoodSensor);
        }
    }

    @Override
    public void periodic()
    {
        _hoodMotorVoltage = Volts.of(_hoodMotor.getMotorOutputVoltage());
        _hoodAngle        = Degrees.of(_hoodSensor.get());
        Voltage voltageOutput = Volts.zero();

        if (_hasSetpoint)
        {
            voltageOutput = Volts.of(_pidController.calculate(_hoodAngle.in(Degrees), _hoodSetpoint.in(Degrees)));
        }

        setHoodMotorVoltage(voltageOutput);
        
        for(var position : HoodPosition.values()){
            
        if(position.getTargetAngle() == null || _hoodAngle.isNear(position.getTargetAngle(), ShooterConstants.HOOD_TOLERANCE))
        {
            _hoodPosition = position;
            break;
        }
        
        }

    }

    @Override
    public void simulationPeriodic()
    {
    }

    public void stop()
    {
        _hasSetpoint = false;
        setHoodMotorVoltage(Volts.zero());
    }

    private void setHoodMotorVoltage(Voltage voltage)
    {
        double clampedVoltage = MathUtil.clamp(voltage.in(Volts), -GeneralConstants.MOTOR_VOLTAGE, GeneralConstants.MOTOR_VOLTAGE);
        _hoodMotor.setVoltage(clampedVoltage);
    }

    public void setHoodPosition(HoodPosition hoodPosition)
    {
        _hoodPosition = hoodPosition;
        Angle targetAngle = hoodPosition.getTargetAngle();
        if (targetAngle == null)
        {
            stop();
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
