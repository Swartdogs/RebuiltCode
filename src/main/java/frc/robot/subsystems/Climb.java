package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

@Logged
public class Climb extends SubsystemBase
{
    public enum ClimbLevel
    {
        L1, L3
    }

    public enum ExtensionState
    {
        Idle, Extending, Retracting
    }

    private final TalonFX  _extendMotor;
    private final TalonFX  _rotateMotor;
    @Logged
    private double         _currentRotation  = 0.0; // TODO: replace with encoder position
    @Logged
    private double         _currentExtension = 0.0; // TODO: replace with encoder position
    @Logged
    private ExtensionState _extensionState   = ExtensionState.Idle;
    @Logged
    private double         _extendOutput     = 0.0;
    @Logged
    private double         _rotateOutput     = 0.0;
    @Logged
    private double         _rotationTarget   = 0.0;
    @Logged
    private boolean        _extended         = false;
    @Logged
    private boolean        _retracted        = true;

    // TODO: Replace with real limit switch inputs.
    private boolean _outSwitch = false;
    private boolean _inSwitch  = true;

    public Climb()
    {
        _extendMotor = new TalonFX(Constants.CAN.CLIMBER_EXTEND);
        _rotateMotor = new TalonFX(Constants.CAN.CLIMBER_ROTATE);

        var extendConfig = new MotorOutputConfigs();
        extendConfig.NeutralMode = NeutralModeValue.Brake;
        extendConfig.Inverted    = InvertedValue.Clockwise_Positive; // TODO: verify extend motor inversion
        _extendMotor.getConfigurator().apply(extendConfig);

        var rotateConfig = new MotorOutputConfigs();
        rotateConfig.NeutralMode = NeutralModeValue.Brake;
        rotateConfig.Inverted    = InvertedValue.CounterClockwise_Positive; // TODO: verify rotate motor inversion
        _rotateMotor.getConfigurator().apply(rotateConfig);
    }

    @Override
    public void periodic()
    {
        // TODO: Read sensors/encoders to update _currentRotation/_currentExtension.
        updateExtensionStateFromLimits();
        updateExtension();
        updateRotation();
        applyOutputs();
    }

    private void updateExtension()
    {
        switch (_extensionState)
        {
            case Idle:
                _extendOutput = 0.0;
                break;

            case Extending:
                _extendOutput = ClimberConstants.EXTEND_OUTPUT;
                break;

            case Retracting:
                _extendOutput = -ClimberConstants.EXTEND_OUTPUT;
                break;

            default:
                _extendOutput = 0.0;
                break;
        }
    }

    private void updateExtensionStateFromLimits()
    {
        boolean atExtendLimit  = _outSwitch;
        boolean atRetractLimit = _inSwitch;

        if (atExtendLimit)
        {
            _extended  = true;
            _retracted = false;

            if (_extensionState == ExtensionState.Extending)
            {
                _extensionState = ExtensionState.Idle;
            }
        }

        if (atRetractLimit)
        {
            _retracted = true;
            _extended  = false;

            if (_extensionState == ExtensionState.Retracting)
            {
                _extensionState = ExtensionState.Idle;
            }
        }
    }

    private void updateRotation()
    {
        if (!_extended)
        {
            _rotateOutput = 0.0;
            return;
        }

        if (Math.abs(_currentRotation - _rotationTarget) <= ClimberConstants.ROTATION_TOLERANCE)
        {
            _rotateOutput = 0.0;
            return;
        }

        _rotateOutput = (_rotationTarget > _currentRotation) ? ClimberConstants.ROTATE_OUTPUT : -ClimberConstants.ROTATE_OUTPUT;
    }

    public void setRotationTarget(ClimbLevel level)
    {
        switch (level)
        {
            case L1:
                _rotationTarget = ClimberConstants.L1_ROTATION;
                break;

            case L3:
                _rotationTarget = ClimberConstants.L3_ROTATION;
                break;

            default:
                break;
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

    public Command getExtendCmd()
    {
        return runOnce(() -> setExtensionState(ExtensionState.Extending));
    }

    public Command getRetractCmd()
    {
        return runOnce(() -> setExtensionState(ExtensionState.Retracting));
    }

    public Command getRotateCmd(ClimbLevel level)
    {
        return runOnce(() -> setRotationTarget(level));
    }

    public Command getStopCmd()
    {
        return runOnce(() -> setExtensionState(ExtensionState.Idle));
    }

    public void setOutSwitch(boolean outSwitch)
    {
        _outSwitch = outSwitch;
    }

    public void setInSwitch(boolean inSwitch)
    {
        _inSwitch = inSwitch;
    }

    public boolean isExtended()
    {
        return _outSwitch;
    }

    public boolean isRetracted()
    {
        return _inSwitch;
    }

    private void applyOutputs()
    {
        _extendMotor.set(_extendOutput);
        _rotateMotor.set(_rotateOutput);
    }
}
