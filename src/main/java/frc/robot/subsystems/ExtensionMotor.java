package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SimulationConstants;

@Logged
public class ExtensionMotor extends SubsystemBase
{
    private final SparkFlex        _extendMotor;
    private final SparkLimitSwitch _outLimitSwitch;
    private final SparkLimitSwitch _inLimitSwitch;
    private final SparkFlexSim     _extensionMotorSim;
    private final Voltage          _extendOutput;
    private final Voltage          _retractVolts;
    private final DCMotor          _neoVortexx;
    @Logged
    private Distance               _currentExtension   = Inches.zero();
    @Logged
    private Voltage                _motorVoltage       = Volts.zero();
    @Logged
    private boolean                _outSwitchTriggered = false;
    @Logged
    private boolean                _inSwitchTriggered  = false;
    private Alert                  _limitSwitchAlert;

    public ExtensionMotor(int CANID, Voltage extendOutput, Voltage retractVolts, double extensionConversionFactor)
    {
        // refer to the following url for more hardware info:
        // https://docs.revrobotics.com/brushless/spark-flex/spark-flex-feature-description/data-port

        _extendMotor    = new SparkFlex(CANID, MotorType.kBrushless);
        _outLimitSwitch = _extendMotor.getForwardLimitSwitch();
        _inLimitSwitch  = _extendMotor.getReverseLimitSwitch();
        _extendOutput   = extendOutput;
        _retractVolts   = retractVolts;

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE);

        var encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(extensionConversionFactor);

        _extendMotor.setVoltage(Volts.zero());
        var limitSwitchConfig = new LimitSwitchConfig();
        limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyOpen).forwardLimitSwitchPosition(IntakeConstants.EXTENSION_MAX_POSITION).forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotorAndSetPosition)
                .reverseLimitSwitchType(Type.kNormallyOpen).reverseLimitSwitchPosition(IntakeConstants.EXTENSION_MIN_POSITION).reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);

        _extendMotor.configure(config.apply(encoderConfig).apply(limitSwitchConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _limitSwitchAlert = new Alert("Both in and out limit switches are triggered for motor CAN ID " + CANID + ".", Alert.AlertType.kWarning);
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
        _outSwitchTriggered = _outLimitSwitch.isPressed();
        _inSwitchTriggered  = _inLimitSwitch.isPressed();
        _limitSwitchAlert.set(_outSwitchTriggered && _inSwitchTriggered);

        _currentExtension = Inches.of(_extendMotor.getEncoder().getPosition());
    }

    @Override
    public void simulationPeriodic()
    {
        _extensionMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        _extensionMotorSim.iterate(_extensionMotorSim.getAppliedOutput() * RadiansPerSecond.of(_neoVortexx.freeSpeedRadPerSec).in(RPM), RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD_SECS);

        Distance position       = Inches.of(_extensionMotorSim.getPosition());
        double   output         = _extensionMotorSim.getAppliedOutput();
        boolean  forwardPressed = position.gte(SimulationConstants.EXTENDED_DISTANCE);
        boolean  reversePressed = position.lte(SimulationConstants.RETRACTED_DISTANCE);

        _extensionMotorSim.getForwardLimitSwitchSim().setPressed(forwardPressed);
        _extensionMotorSim.getReverseLimitSwitchSim().setPressed(reversePressed);

        if ((forwardPressed && output > 0) || (reversePressed && output < 0))
        {
            _extensionMotorSim.setAppliedOutput(0);
        }
    }

    public void extend(boolean finalState)
    {
        _extendMotor.setVoltage(finalState ? _extendOutput : _retractVolts);
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
        return _outSwitchTriggered;
    }

    public boolean isRetracted()
    {
        return _inSwitchTriggered;
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
