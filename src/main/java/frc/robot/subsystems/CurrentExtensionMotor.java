package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SimulationConstants;

@Logged
public class CurrentExtensionMotor extends SubsystemBase
{
    private final SparkFlex    _extendMotor;
    private final SparkFlexSim _extensionMotorSim;
    private final Voltage      _extendOutput;
    private final Voltage      _retractVolts;
    private final DCMotor      _neoVortexx;
    @Logged
    private Current            _current          = Amps.zero();
    @Logged
    private Distance           _currentExtension = Inches.zero();
    @Logged
    private Voltage            _motorVoltage     = Volts.zero();
    @Logged
    private boolean            _extended         = false;
    @Logged
    private boolean            _retracted        = false;

    public CurrentExtensionMotor(int CANID, Voltage extendOutput, Voltage retractVolts, Per<DistanceUnit, AngleUnit> extensionConversionFactor)
    {
        // refer to the following url for more hardware info:
        // https://docs.revrobotics.com/brushless/spark-flex/spark-flex-feature-description/data-port

        _extendMotor  = new SparkFlex(CANID, MotorType.kBrushless);
        _extendOutput = extendOutput;
        _retractVolts = retractVolts;

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit((int)IntakeConstants.CURRENT_LIMIT.in(Amps)).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));

        var encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(extensionConversionFactor.in(Inches.per(Rotation)));

        _extendMotor.configure(config.apply(encoderConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _extendMotor.setVoltage(Volts.zero());

        if (RobotBase.isReal())
        {
            _neoVortexx        = null;
            _extensionMotorSim = null;
        }
        else
        {
            _neoVortexx        = DCMotor.getNeoVortex(1);
            _extensionMotorSim = new SparkFlexSim(_extendMotor, _neoVortexx);
        }
    }

    @Override
    public void periodic()
    {
        // TODO: Read sensors/encoders to update _currentRotation/_currentExtension.
        _currentExtension = Inches.of(_extendMotor.getEncoder().getPosition());
        _motorVoltage     = Volts.of(_extendMotor.getAppliedOutput() * _extendMotor.getBusVoltage());
        _current          = Amps.of(_extendMotor.getOutputCurrent());

        if (_current.abs(Amps) > IntakeConstants.EXTENSION_PEAK_CURRENT.in(Amps))
        {
            _extendMotor.stopMotor();
            _extended  = _motorVoltage.gt(Volts.zero());
            _retracted = _motorVoltage.lt(Volts.zero());
        }
    }

    @Override
    public void simulationPeriodic()
    {
        _extensionMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _extensionMotorSim.iterate(_extensionMotorSim.getAppliedOutput() * RadiansPerSecond.of(_neoVortexx.freeSpeedRadPerSec).in(RPM), RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD.in(Seconds));

        Distance      position       = Inches.of(_extensionMotorSim.getPosition());
        Dimensionless output         = Value.of(_extensionMotorSim.getAppliedOutput());
        boolean       forwardPressed = position.gte(SimulationConstants.EXTENDED_DISTANCE);
        boolean       reversePressed = position.lte(SimulationConstants.RETRACTED_DISTANCE);
        var           current        = Amps.zero();

        if (output.gt(Value.zero()))
        {
            if (forwardPressed == false)
            {
                current = Amps.of(_neoVortexx.freeCurrentAmps);

            }
            else
            {
                current = Amps.of(100);
            }
        }
        else if (output.lt(Value.zero()))
        {
            if (reversePressed == false)
            {
                current = Amps.of(-_neoVortexx.freeCurrentAmps);
            }
            else
            {
                current = Amps.of(-100);
            }
        }

        _extensionMotorSim.setMotorCurrent(current.in(Amps));
    }

    public void extend(boolean finalState)
    {
        var volts = Volts.zero();

        if (finalState && !_extended)
        {
            volts      = _extendOutput;
            _retracted = false;
        }
        else if (!finalState && !_retracted)
        {
            volts     = _retractVolts;
            _extended = false;
        }
        _extendMotor.setVoltage(volts);
    }

    public Voltage getMotorVoltage()
    {
        return Volts.of(_extendMotor.getAppliedOutput() * _extendMotor.getBusVoltage());
    }

    public Distance getMotorPosition()
    {
        return _currentExtension;
    }

    public boolean isExtended()
    {
        return _extended;
    }

    public boolean isRetracted()
    {
        return _retracted;
    }

    /* COMMANDS */
    public Command getExtendCmd()
    {
        return runOnce(() -> extend(true));
    }

    public Command getRetractCmd()
    {
        return runOnce(() -> extend(false));
    }
}
