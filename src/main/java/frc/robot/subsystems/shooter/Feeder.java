package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Feeder extends SubsystemBase
{
    private final SparkFlex _feederMotor;
    private final SparkSim  _feederMotorSim;
    @Logged
    private Voltage         _feederMotorVoltage = Volts.of(0.0);

    public Feeder()
    {
        _feederMotor = new SparkFlex(CANConstants.FEEDER_MOTOR, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.FEEDER_CURRENT_LIMIT).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE);

        _feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        if (RobotBase.isReal())
        {
            _feederMotorSim = null;
        }
        else
        {
            _feederMotorSim = new SparkSim(_feederMotor, DCMotor.getNeoVortex(1));
        }
    }

    public void periodic()
    {
        _feederMotorVoltage = Volts.of(_feederMotor.getAppliedOutput() * _feederMotor.getBusVoltage());
    }

    public void simulationPeriodic()
    {
        if (_feederMotorSim == null) return;
        _feederMotorSim.iterate(0., RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD_SECS);
    }

    public void set(boolean on)
    {
        _feederMotor.setVoltage(on ? ShooterConstants.FEEDER_VOLTAGE : 0.0);
    }
}
