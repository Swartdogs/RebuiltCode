package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;

@Logged
public class ExtensionMotor extends SubsystemBase
{
    public enum ExtensionState
    {
        Idle, Extending, Retracting
    }

    private final SparkFlex        _extendMotor;
    private final SparkLimitSwitch _outLimitSwitch;
    private final SparkLimitSwitch _inLimitSwitch;
    private final Voltage          _extendOutput;
    private final Voltage          _retractVolts;
    private final Distance         _minExtension;
    private final Distance         _maxExtension;
    @Logged
    private ExtensionState         _extensionState;
    @Logged
    private Distance               _currentExtension;
    @Logged
    private Voltage                _motorVoltage;
    @Logged
    private boolean                _extended;
    @Logged
    private boolean                _retracted;
    @Logged
    private boolean                _outSwitchTriggered;
    @Logged
    private boolean                _inSwitchTriggered;

    public ExtensionMotor(int CANID, Voltage extendOutput, Voltage retractVolts, Distance minExtension, Distance maxExtension)
    {
        _extendMotor    = new SparkFlex(CANID, MotorType.kBrushless);
        _outLimitSwitch = _extendMotor.getForwardLimitSwitch();
        _inLimitSwitch  = _extendMotor.getReverseLimitSwitch();
        _extendOutput   = extendOutput;
        _retractVolts   = retractVolts;
        _minExtension   = minExtension;
        _maxExtension   = maxExtension;
        _extensionState = ExtensionState.Idle;

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE);
        _extendMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _currentExtension = Inches.of(_extendMotor.getEncoder().getPosition()); // TODO: replace with encoder position

        // TODO: Replace with real limit switch inputs.
        _inSwitchTriggered  = _inLimitSwitch.isPressed();
        _outSwitchTriggered = _outLimitSwitch.isPressed();

        _extended  = _outSwitchTriggered;
        _retracted = _inSwitchTriggered;
    }

    @Override
    public void periodic()
    {
        // TODO: Read sensors/encoders to update _currentRotation/_currentExtension.
        _outSwitchTriggered = _outLimitSwitch.isPressed();
        _inSwitchTriggered  = _inLimitSwitch.isPressed();
        updateExtensionStateFromLimits();
        updateMotorVoltage();
        setExtensionVoltage(_motorVoltage);
        _currentExtension = Inches.of(_extendMotor.getEncoder().getPosition());
        _currentExtension = Inches.of(MathUtil.clamp(_currentExtension.in(Inches), _minExtension.in(Inches), _maxExtension.in(Inches)));
    }

    private void updateMotorVoltage()
    {
        switch (_extensionState)
        {
            case Idle:
                stop();
                break;

            case Extending:
                _motorVoltage = _extendOutput;
                break;

            case Retracting:
                _motorVoltage = _retractVolts;
                break;

            default:
                stop();
                break;
        }
    }

    private void updateExtensionStateFromLimits()
    {
        if (_outSwitchTriggered)
        {
            _extended  = true;
            _retracted = false;

            if (_extensionState == ExtensionState.Extending)
            {
                _extensionState = ExtensionState.Idle;
            }
        }

        if (_inSwitchTriggered)
        {
            _retracted = true;
            _extended  = false;

            if (_extensionState == ExtensionState.Retracting)
            {
                _extensionState = ExtensionState.Idle;
            }
        }
    }

    public void setExtensionState(ExtensionState state)
    {
        switch (_extensionState)
        {
            case Idle:
                if (state == ExtensionState.Extending && !_extended)
                {
                    _extensionState = state;
                }
                else if (state == ExtensionState.Retracting && !_retracted)
                {
                    _extensionState = state;
                }
                break;

            case Extending:
                if (state == ExtensionState.Idle || state == ExtensionState.Retracting)
                {
                    _extensionState = state;
                }
                break;

            case Retracting:
                if (state == ExtensionState.Idle || state == ExtensionState.Extending)
                {
                    _extensionState = state;
                }
                break;

            default:
                break;
        }
    }

    public void stop()
    {
        setExtensionState(ExtensionState.Idle);
        setExtensionVoltage(Volts.zero());
    }

    public void setExtensionVoltage(Voltage volts)
    {
        _extendMotor.setVoltage(volts);
    }

    public void setOutSwitch(boolean outSwitch)
    {
        _outSwitchTriggered = outSwitch;
    }

    public void setInSwitch(boolean inSwitch)
    {
        _inSwitchTriggered = inSwitch;
    }

    public Voltage getMotorVoltage()
    {
        return Volts.of(_extendMotor.getAppliedOutput() * _extendMotor.getBusVoltage());
    }

    public Voltage getAppliedOutput()
    {
        return Volts.of(_extendMotor.getAppliedOutput());
    }

    public Voltage getBusVoltage()
    {
        return Volts.of(_extendMotor.getBusVoltage());
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
        return runOnce(() -> setExtensionState(ExtensionState.Extending));
    }

    public Command getRetractCmd()
    {
        return runOnce(() -> setExtensionState(ExtensionState.Retracting));
    }

    public Command getStopCmd()
    {
        return runOnce(() -> setExtensionState(ExtensionState.Idle));
    }
}
