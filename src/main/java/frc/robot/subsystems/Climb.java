package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class Climb extends SubsystemBase 
{
    public enum ClimbLevel
    {
        L1, L3; // TODO
    }
    
    public enum ExtensionState
    {
        Idle, Extending, Retracting
    }

    @Logged
    private double _currentRotation; 
    @Logged
    private double _currentExtension; 
    @Logged
    private ExtensionState _extenstionState; 
    @Logged
    private boolean _extended; 
    @Logged
    private boolean _retracted; 

    public Climb() 
    {
        _extenstionState = ExtensionState.Idle; 
        _currentRotation = 0.0; 
        _currentExtension = 0.0; 
        _extended = false; 
        _retracted = true; 
    }

    public void periodic() 
    {
        updateExtension();   
        _extended = isExtended(); 
        _retracted = isRetracted(); 

        if (_extended || _retracted) // hopefully both aren't true at the same time. if so, we use xor (^)
        {
            _extenstionState = ExtensionState.Idle; 
        }
    }

    public void updateExtension()
    {
        switch (_extenstionState) {
            case Idle:
                // tell the motor to stop
                break;

            case Extending:
                // tell the motor to extend 
                // update _currentExtension
                break; 
            
            case Retracting:
                // tell the motor to retract 
                // update _currentExtension 
                break; 
        
            default:
                break;
        }
    }

    public void setExtensionState(ExtensionState state)
    {
        switch (_extenstionState) {
            case Idle:
                if (state == ExtensionState.Extending && !_extended) 
                {
                    _extenstionState = state; 
                }
                else if (state == ExtensionState.Retracting && !_retracted)
                {
                    _extenstionState = state; 
                }
                break;

            case Extending:
                if (state == ExtensionState.Idle && _extended) 
                {
                    _extenstionState = state; 
                }
                else if (state == ExtensionState.Retracting)
                {
                    _extenstionState = state; 
                }
                break;

            case Retracting:
                if (state == ExtensionState.Idle && _retracted) 
                    {
                        _extenstionState = state; 
                    }
                    else if (state == ExtensionState.Extending)
                    {
                        _extenstionState = state; 
                    }
                break;
        
            default:
                break;
        }
    }

    public void rotateClimber(ClimbLevel level)
    {
        // TODO 
        switch (level) 
        {
            case L1:
                // turn using ClimberConstants.L1_ROTATION
                break;

            case L3:
                // turn using ClimberConstants.L3_ROTATION
                break;
        
            default:
                break;
        }
    }

    public Command getExtendCmd()
    {
        return runOnce(); // TODO 
    }

    public Command getRotateCmd(ClimbLevel level)
    {
        return runOnce(
            () -> rotateClimber(level)
        ); // TODO 
    }

    public boolean isExtended() 
    {
        return _currentExtension >= ClimberConstants.EXTENSION_THRESHOLD;
    }

    public boolean isRetracted() 
    {
        return _currentExtension <= ClimberConstants.RETRACTION_THRESHOLD;
    }
}
