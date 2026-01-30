package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterFlywheel extends SubsystemBase
{
    private final SparkFlex                 _leadMotor;
    private final SparkFlex                 _followMotor;
    private final SparkClosedLoopController _closedLoopController;
    private final RelativeEncoder           _encoder;
    private final SparkSim                  _leadMotorSim;
    private AngularVelocity                 _velocity       = RPM.of(0.0);
    private AngularVelocity                 _velocityTarget = RPM.of(0.0);

    public ShooterFlywheel()
    {
        _leadMotor   = new SparkFlex(Constants.CAN.FLYWHEEL_LEAD, MotorType.kBrushless);
        _followMotor = new SparkFlex(Constants.CAN.FLYWHEEL_FOLLOW, MotorType.kBrushless);

        if (RobotBase.isReal())
        {
            _leadMotor.setCANTimeout(250);
            _followMotor.setCANTimeout(250);
        }

        var config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit((int)Constants.Shooter.FLYWHEEL_CURRENT_LIMIT.in(Amps)).voltageCompensation(Constants.General.MOTOR_VOLTAGE.in(Volts));

        config.inverted(false);
        config.closedLoop.p(Constants.Shooter.FLYWHEEL_KP).d(Constants.Shooter.FLYWHEEL_KD);
        config.closedLoop.feedForward.kS(Constants.Shooter.FLYWHEEL_KS).kV(Constants.Shooter.FLYWHEEL_KV).kA(Constants.Shooter.FLYWHEEL_KA);
        _leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(_leadMotor, true);
        _followMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _closedLoopController = _leadMotor.getClosedLoopController();
        _encoder              = _leadMotor.getEncoder();

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
        _velocity = RPM.of(_encoder.getVelocity());
    }

    @Override
    public void simulationPeriodic()
    {
        _leadMotorSim.iterate(getVelocity(), RoboRioSim.getVInVoltage(), Constants.General.LOOP_PERIOD.in(Seconds));
    }

    public void setVelocity(AngularVelocity velocity)
    {
        _velocityTarget = velocity;
        _closedLoopController.setSetpoint(_velocityTarget.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void stop()
    {
        _velocityTarget = RPM.of(0.0);
        _leadMotor.setVoltage(0);
    }

    public boolean atSpeed()
    {
        if (_velocityTarget.in(RPM) <= 0) return true;

        double error = getVelocity().minus(_velocityTarget);
        return error <= _velocityTarget * Constants.Shooter.FLYWHEEL_TOLERANCE;
    }

    public AngularVelocity getVelocity()
    {
        return _velocity;
    }
}
