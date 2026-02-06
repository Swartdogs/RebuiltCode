package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Hood extends SubsystemBase
{
    /**
     * Enumerates positions of Shoot, Pass and, Undefined. A private constructor is
     * used to give an Angle to each enumeration
     *
     * @param angle The angle for each action, obtained from constants, undefined
     *              though is null.
     */
    public enum HoodPosition
    {
        Shoot(ShooterConstants.HOOD_SHOOT_ANGLE), Pass(ShooterConstants.HOOD_PASS_ANGLE), Undefined(null);

        private Angle _angle;

        private HoodPosition(Angle angle)
        {
            _angle = angle;
        }

        /**
         * Returns true if the angle is null for simulation purposes. Otherwise, returns
         * if the double absolute difference is less than the tolerance.
         *
         * @param  angle Takes in the current angle desired as an angle (intended to be
         *               used with the enumerated angle values, once more per
         *               constants).
         *
         * @return       If the absolute difference between target and current angle
         *               exceed tolerance is true.
         */
        public boolean atPosition(Angle angle)
        {
            if (_angle == null) return true;
            double difference = angle.minus(_angle).abs(Degrees);
            return difference < ShooterConstants.HOOD_TOLERANCE.in(Degrees);
        }
    }

    private final WPI_VictorSPX       _hoodMotor;
    private final AnalogPotentiometer _hoodSensor;
    private final PIDController       _pidController;
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
        _hoodSensor       = new AnalogPotentiometer(AIOConstants.HOOD_POTENTIOMETER, ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MIN_ANGLE);
        _hoodMotorVoltage = Volts.of(0.0);
        _hoodAngle        = Degrees.of(0.0);
        _hoodPosition     = HoodPosition.Undefined;
        _pidController    = new PIDController(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD);
        _pidController.setTolerance(ShooterConstants.HOOD_TOLERANCE.in(Degrees));
        _hoodSetpoint = null;
    }

    public void periodic()
    {
        _hoodMotorVoltage = Volts.of(_hoodMotor.getMotorOutputVoltage());
        _hoodAngle        = Degrees.of(_hoodSensor.get());

        if (_hoodSetpoint != null)
        {
            _hoodMotor.setVoltage(_pidController.calculate(_hoodAngle.in(Degrees)));
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

    public void stop()
    {
        _hoodMotor.setVoltage(0.0);
        _hoodSetpoint = null;
    }

    public void setHoodPosition(HoodPosition hoodPosition)
    {
        if (hoodPosition == HoodPosition.Undefined) return;

        _hoodSetpoint = switch (hoodPosition)
        {
            case Shoot -> ShooterConstants.HOOD_SHOOT_ANGLE;
            case Pass -> ShooterConstants.HOOD_PASS_ANGLE;
            default -> null;
        };
        _pidController.setSetpoint(_hoodSetpoint.in(Degrees));
    }

    public HoodPosition getHoodPosition()
    {
        return _hoodPosition;
    }
}
