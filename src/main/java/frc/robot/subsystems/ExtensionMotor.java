package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;

@Logged
public class ExtensionMotor extends SubsystemBase
{
    private final SparkFlex        _extendMotor;
    private final SparkLimitSwitch _outLimitSwitch;
    private final SparkLimitSwitch _inLimitSwitch;
    private final Voltage          _extendOutput;
    private final Voltage          _retractVolts;
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
        _extendMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(extensionConversionFactor);

        _extendMotor.setVoltage(Volts.zero());

        _limitSwitchAlert = new Alert("Both in and out limit switches are triggered for motor CAN ID " + CANID + ".", Alert.AlertType.kWarning);
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
