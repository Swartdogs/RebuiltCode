package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Feeder
{
    private final SparkFlex    _feederMotor;
    private final SparkFlexSim _feederMotorSim;
    @Logged
    private Voltage            _feederMotorVoltage = Volts.of(0.0);

    public Feeder()
    {
        _feederMotor = new SparkFlex(CANConstants.FEEDER_MOTOR, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit((int)ShooterConstants.FEEDER_CURRENT_LIMIT.in(Amps)).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        _feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isReal())
        {
            _feederMotorSim = null;
        }
        else
        {
            _feederMotorSim = new SparkFlexSim(_feederMotor, DCMotor.getNeoVortex(1));
        }
    }

    public void periodic()
    {
        _feederMotorVoltage = Volts.of(_feederMotor.getAppliedOutput() * _feederMotor.getBusVoltage());
    }

    public void set(boolean on)
    {
        Voltage targetVoltage = on ? ShooterConstants.FEEDER_VOLTAGE : Volts.zero();
        _feederMotor.setVoltage(targetVoltage.in(Volts));

        if (RobotBase.isSimulation())
        {
            _feederMotorSim.setAppliedOutput(targetVoltage.in(Volts) / RoboRioSim.getVInVoltage());
        }
    }
}
