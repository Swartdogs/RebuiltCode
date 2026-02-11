package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimberConstants;

@Logged
public class Climber extends SubsystemBase
{
    public enum ClimbLevel
    {
        L1, L3
    }

    private final ExtensionMotor _extender;
    private final TalonFX        _rotateMotor;
    @Logged
    private double               _currentRotation = 0.0; // TODO: replace with encoder position
    @Logged
    private double               _rotateOutput    = 0.0;
    @Logged
    private double               _rotationTarget  = 0.0;

    public Climber()
    {
        _extender = new ExtensionMotor(CANConstants.CLIMBER_EXTEND, ClimberConstants.EXTEND_OUTPUT, ClimberConstants.RETRACT_VOLTAGE, Inches.zero(), Inches.zero());

        _rotateMotor = new TalonFX(CANConstants.CLIMBER_ROTATE);

        var rotateConfig = new MotorOutputConfigs();
        rotateConfig.NeutralMode = NeutralModeValue.Brake;
        rotateConfig.Inverted    = InvertedValue.CounterClockwise_Positive; // TODO: verify rotate motor inversion
        _rotateMotor.getConfigurator().apply(rotateConfig);
    }

    @Override
    public void periodic()
    {
        // TODO: Read sensors/encoders to update _currentRotation.
        updateRotation();
        _rotateMotor.set(_rotateOutput);
    }

    private void updateRotation()
    {
        if (!_extender.isExtended())
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

    public Command getRotateCmd(ClimbLevel level)
    {
        return runOnce(() -> setRotationTarget(level));
    }
}
