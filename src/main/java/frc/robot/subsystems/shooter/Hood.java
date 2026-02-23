package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.MotorHook;
import frc.robot.TestHook;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.MeasureUtil;

import com.ctre.phoenix.motorcontrol.NeutralMode;

@Logged
public class Hood
{
    private static final String kSettingsPrefix = "Dashboard/Dashboard Settings/";

    public enum HoodPosition
    {
        Shoot(ShooterConstants.HOOD_SHOOT_ANGLE), Pass(ShooterConstants.HOOD_PASS_ANGLE), Undefined(null);

        public final Angle targetAngle;

        private HoodPosition(Angle targetAngle)
        {
            this.targetAngle = targetAngle;
        }
    }

    private final WPI_VictorSPX       _hoodMotor;
    private final DutyCycleEncoder    _hoodSensor;
    private final DutyCycleEncoderSim _hoodSensorSim;
    private final PIDController       _pidController;
    private final DCMotor             _motorModel;
    private Angle                     _simAngleDeg;
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
        _hoodSensor       = new DutyCycleEncoder(DIOConstants.HOOD_ENCODER, ShooterConstants.HOOD_MAX_ANGLE.minus(ShooterConstants.HOOD_MIN_ANGLE).in(Degrees), ShooterConstants.HOOD_MIN_ANGLE.in(Degrees));
        _pidController    = new PIDController(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD);
        _hoodMotorVoltage = Volts.zero();
        _hoodAngle        = Degrees.zero();
        _hoodPosition     = HoodPosition.Undefined;
        _hoodSetpoint     = Degrees.zero();
        _hasSetpoint      = false;
        _simAngleDeg      = ShooterConstants.HOOD_PASS_ANGLE;

        initializeTuningPreferences();

        _pidController.setTolerance(ShooterConstants.HOOD_TOLERANCE.in(Degrees));

        // Dashboard
        SmartDashboard.putData("Hood PID", _pidController);

        // Real vs. Simulation
        if (RobotBase.isReal())
        {
            _hoodMotor.configFactoryDefault();
            _hoodMotor.setNeutralMode(NeutralMode.Brake);
            _hoodMotor.setInverted(false); // TODO: Check direction
            _hoodSensorSim = null;
            _motorModel    = null;
        }
        else
        {
            _hoodSensorSim = new DutyCycleEncoderSim(_hoodSensor);
            _motorModel    = GeneralConstants.WINDOW_MOTOR;
        }
    }

    public void periodic()
    {
        _pidController.setTolerance(getToleranceAngle().in(Degrees));
        _hoodMotorVoltage = Volts.of(_hoodMotor.getMotorOutputVoltage());
        _hoodAngle        = Degrees.of(_hoodSensor.get());
        Voltage voltageOutput = Volts.zero();

        if (_hasSetpoint)
        {
            voltageOutput = Volts.of(_pidController.calculate(_hoodAngle.in(Degrees), _hoodSetpoint.in(Degrees)));
        }

        setHoodMotorVoltage(voltageOutput);

        for (var position : HoodPosition.values())
        {
            Angle target = getTargetAngle(position);
            if (target == null || _hoodAngle.isNear(target, getToleranceAngle()))
            {
                _hoodPosition = position;
                break;
            }
        }
    }

    public void simulationPeriodic()
    {
        var percentOutput = Volts.of(_hoodMotor.getMotorOutputVoltage()).div(GeneralConstants.MOTOR_VOLTAGE);
        var delta         = RadiansPerSecond.of(_motorModel.freeSpeedRadPerSec).times(percentOutput).times(ShooterConstants.HOOD_GEAR_RATIO).times(GeneralConstants.LOOP_PERIOD);
        _simAngleDeg = MeasureUtil.clamp(_simAngleDeg.plus(delta), ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);
        _hoodSensorSim.set(_simAngleDeg.in(Degrees));
    }

    public void stop()
    {
        _hasSetpoint = false;
        setHoodMotorVoltage(Volts.zero());
    }

    private void setHoodMotorVoltage(Voltage voltage)
    {
        _hoodMotor.setVoltage(MeasureUtil.clamp(voltage, GeneralConstants.MOTOR_VOLTAGE.unaryMinus(), GeneralConstants.MOTOR_VOLTAGE));
    }

    public void setHoodPosition(HoodPosition hoodPosition)
    {
        if (hoodPosition == HoodPosition.Undefined)
        {
            stop();
            return;
        }

        Angle targetAngle = getTargetAngle(hoodPosition);
        _hoodSetpoint = MeasureUtil.clamp(targetAngle, getMinAngle(), getMaxAngle());
        _hasSetpoint  = true;
        _pidController.setSetpoint(_hoodSetpoint.in(Degrees));
    }

    public boolean atSetpoint()
    {
        return _hasSetpoint && _hoodAngle.isNear(_hoodSetpoint, getToleranceAngle());
    }

    public HoodPosition getHoodPosition()
    {
        return _hoodPosition;
    }

    private class HoodHook extends MotorHook
    {
        @Override
        public void stop()
        {
            Hood.this.stop();
        }

        @Override
        public void setRate(double rate)
        {
            setHoodMotorVoltage(GeneralConstants.MOTOR_VOLTAGE.times(rate * _polarity));
        }
    }

    public TestHook getHook()
    {
        return new HoodHook();
    }

    private static void initPreference(String key, double value)
    {
        Preferences.initDouble(kSettingsPrefix + key, value);
    }

    private static void initializeTuningPreferences()
    {
        initPreference("Hood Min Angle", ShooterConstants.HOOD_MIN_ANGLE.in(Degrees));
        initPreference("Hood Max Angle", ShooterConstants.HOOD_MAX_ANGLE.in(Degrees));
        initPreference("Hood Shoot Angle", ShooterConstants.HOOD_SHOOT_ANGLE.in(Degrees));
        initPreference("Hood Pass Angle", ShooterConstants.HOOD_PASS_ANGLE.in(Degrees));
        initPreference("Hood Tolerance", ShooterConstants.HOOD_TOLERANCE.in(Degrees));
    }

    private static Angle getMinAngle()
    {
        return Degrees.of(Preferences.getDouble(kSettingsPrefix + "Hood Min Angle", ShooterConstants.HOOD_MIN_ANGLE.in(Degrees)));
    }

    private static Angle getMaxAngle()
    {
        return Degrees.of(Preferences.getDouble(kSettingsPrefix + "Hood Max Angle", ShooterConstants.HOOD_MAX_ANGLE.in(Degrees)));
    }

    private static Angle getShootAngle()
    {
        return Degrees.of(Preferences.getDouble(kSettingsPrefix + "Hood Shoot Angle", ShooterConstants.HOOD_SHOOT_ANGLE.in(Degrees)));
    }

    private static Angle getPassAngle()
    {
        return Degrees.of(Preferences.getDouble(kSettingsPrefix + "Hood Pass Angle", ShooterConstants.HOOD_PASS_ANGLE.in(Degrees)));
    }

    private static Angle getToleranceAngle()
    {
        return Degrees.of(Preferences.getDouble(kSettingsPrefix + "Hood Tolerance", ShooterConstants.HOOD_TOLERANCE.in(Degrees)));
    }

    private static Angle getTargetAngle(HoodPosition hoodPosition)
    {
        return switch (hoodPosition)
        {
            case Shoot -> getShootAngle();
            case Pass -> getPassAngle();
            case Undefined -> null;
        };
    }
}
