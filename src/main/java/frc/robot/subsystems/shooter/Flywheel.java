package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

public class Flywheel extends SubsystemBase
{
    private final SparkFlex                 _leadMotor;
    private final SparkFlex                 _followMotor;
    private final SparkClosedLoopController _closedLoopController;
    private final RelativeEncoder           _flywheelEncoder;
    private final SparkSim                  _leadMotorSim;
    private AngularVelocity                 _velocity;
    private AngularVelocity                 _targetVelocity;
    private Voltage                         _flywheelVoltage;

    public Flywheel()
    {
        _leadMotor   = new SparkFlex(CANConstants.FLYWHEEL_LEAD, MotorType.kBrushless);
        _followMotor = new SparkFlex(CANConstants.FLYWHEEL_FOLLOW, MotorType.kBrushless);

        if (RobotBase.isReal())
        {
            _leadMotor.setCANTimeout(250);
            _followMotor.setCANTimeout(250);
        }

        var config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE);

        config.inverted(false);
        config.closedLoop.p(ShooterConstants.FLYWHEEL_KP).d(ShooterConstants.FLYWHEEL_KD);
        config.closedLoop.feedForward.kS(ShooterConstants.FLYWHEEL_KS).kV(ShooterConstants.FLYWHEEL_KV).kA(ShooterConstants.FLYWHEEL_KA);
        _leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(_leadMotor, true);
        _followMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _closedLoopController = _leadMotor.getClosedLoopController();
        _flywheelEncoder      = _leadMotor.getEncoder();

        _velocity        = RPM.zero();
        _targetVelocity  = null;
        _flywheelVoltage = Volts.zero();

        if (RobotBase.isReal())
        {
            _leadMotor.setCANTimeout(0);
            _followMotor.setCANTimeout(0);
            _leadMotorSim = null;
        }
        else
        {
            _leadMotorSim = new SparkSim(_leadMotor, DCMotor.getNeoVortex(2));
        }
    }

    @Override
    public void periodic()
    {
        _velocity        = RPM.of(_flywheelEncoder.getVelocity());
        _flywheelVoltage = Volts.of(_leadMotor.getAppliedOutput() * _leadMotor.getBusVoltage());
    }

    @Override
    public void simulationPeriodic()
    {
        if (_leadMotorSim == null) return;
        _leadMotorSim.iterate(getVelocity().in(RPM), RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD_SECS);
    }

    public void setVelocity(AngularVelocity targetVelocity)
    {
        if (_targetVelocity == null)
        {
            stop();
            return;
        }
        _targetVelocity = targetVelocity;
        _closedLoopController.setSetpoint(_targetVelocity.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void stop()
    {
        _targetVelocity = null;
        _leadMotor.setVoltage(0.0);
    }

    public boolean atSpeed()
    {
        if (null == _targetVelocity) return true;
        return getVelocity().isNear(_targetVelocity, ShooterConstants.FLYWHEEL_TOLERANCE);
    }

    public AngularVelocity getVelocity()
    {
        return _velocity;
    }
}
