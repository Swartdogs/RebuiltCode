package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Flywheel extends SubsystemBase
{
    public Command stopFlywheel()
    {
        return runOnce(this::stop);
    }

    public Command setFlywheelVelocity(AngularVelocity velocity)
    {
        return runOnce(() -> setVelocity(velocity));
    }

    private final SparkFlex                 _leadMotor;
    private final SparkFlex                 _followMotor;
    private final SparkClosedLoopController _closedLoopController;
    private final FlywheelSim               _flywheelSim;
    private final RelativeEncoder           _flywheelEncoder;
    private final SparkFlexSim              _leadMotorSim;
    @Logged
    private AngularVelocity                 _velocity        = RPM.zero();
    @Logged
    private AngularVelocity                 _targetVelocity  = RPM.zero();
    @Logged
    private Voltage                         _flywheelVoltage = Volts.zero();

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
        config.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE);
        config.closedLoop.p(ShooterConstants.FLYWHEEL_KP).d(ShooterConstants.FLYWHEEL_KD);
        config.closedLoop.feedForward.kS(ShooterConstants.FLYWHEEL_KS).kV(ShooterConstants.FLYWHEEL_KV).kA(ShooterConstants.FLYWHEEL_KA);
        _leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(_leadMotor, true);
        _followMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _closedLoopController = _leadMotor.getClosedLoopController();
        _flywheelEncoder      = _leadMotor.getEncoder();

        if (RobotBase.isReal())
        {
            _leadMotor.setCANTimeout(0);
            _followMotor.setCANTimeout(0);
            _leadMotorSim = null;
            _flywheelSim  = null;
        }
        else
        {
            var gearbox = DCMotor.getNeoVortex(2);
            _leadMotorSim = new SparkFlexSim(_leadMotor, DCMotor.getNeoVortex(2));
            _flywheelSim  = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, 0.001, 1), gearbox);
        }
    }

    @Override
    public void periodic()
    {
        double appliedOutput = _leadMotor.getAppliedOutput();
        double busVoltage    = _leadMotor.getBusVoltage();
        _velocity        = RPM.of(_flywheelEncoder.getVelocity());
        _flywheelVoltage = Volts.of(appliedOutput * busVoltage);
    }

    @Override
    public void simulationPeriodic()
    {
        _flywheelSim.setInputVoltage(_flywheelVoltage.in(Volts));
        _flywheelSim.update(GeneralConstants.LOOP_PERIOD_SECS);
        var velocity = _flywheelSim.getAngularVelocity();
        _leadMotorSim.getRelativeEncoderSim().setVelocity(velocity.in(RPM));
        _leadMotorSim.iterate(velocity.in(RPM), RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD_SECS);
    }

    public void setVelocity(AngularVelocity targetVelocity)
    {
        if (targetVelocity == null)
        {
            stop();
            return;
        }
        _targetVelocity = targetVelocity;
        _closedLoopController.setSetpoint(_targetVelocity.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void stop()
    {
        _targetVelocity = RPM.zero();
        _leadMotor.setVoltage(0.0);
    }

    public boolean atSpeed()
    {
        if (_targetVelocity == RPM.zero()) return true;
        return getVelocity().isNear(_targetVelocity, ShooterConstants.FLYWHEEL_TOLERANCE);
    }

    public AngularVelocity getVelocity()
    {
        return _velocity;
    }
}
