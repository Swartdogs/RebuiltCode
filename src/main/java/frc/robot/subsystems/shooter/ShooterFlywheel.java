package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants; //TODO: Update constants as needed.
    /*Hardware
The following motors and sensors will need to be added to the subsystem for controlling the hardware

Motors
Flywheel (2x NEO Vortex motors driven by SPARK Flex motor controllers)
Hood (1x Motor of undetermined type)
Turret (1x Kraken x44 driven by a TalonFX motor controller)
Sensors
Hood angle absolute encoder (1x sensor of undetermined type)
Turret angle absolute encoder (1x sensor of undetermined type)
Camera (1x Limelight) */
public class ShooterFlywheel extends SubsystemBase
{
    private final ShooterFlywheelIO                 _io;
    private final ShooterFlywheelIOInputsAutoLogged _inputs                = new ShooterFlywheelIOInputsAutoLogged();
    private double                                  _maxSpeed              = Constants.ShooterFlywheel.MAX_FLYWHEEL_SPEED * Constants.General.MAX_NEO_SPEED;
    private double                                  _intakeVolts           = Constants.ShooterFlywheel.FLYWHEEL_INTAKE_SPEED * Constants.General.MOTOR_VOLTAGE;
    private double                                  _velocityRange         = Constants.ShooterFlywheel.VELOCITY_RANGE;
    private Double                                  _upperVelocitySetpoint = null;
    private Double                                  _lowerVelocitySetpoint = null;

    public ShooterFlywheel(ShooterFlywheelIO flywheelIO)
    {
        _io = flywheelIO;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Shooter/Flywheel", _inputs);
    }

    public void setUpperVelocity(double upperVelocity)
    {
        _upperVelocitySetpoint = MathUtil.clamp(upperVelocity, 0, _maxSpeed);
        Logger.recordOutput("Shooter/Flywheel/UpperVelocitySetpoint", _upperVelocitySetpoint);
        _io.setUpperVelocity(_upperVelocitySetpoint);
    }


    public void setVelocity(double upperVelocity, double lowerVelocity)
    {
        setUpperVelocity(upperVelocity);
        setLowerVelocity(lowerVelocity);
    }

    public void stop()
    {
        _upperVelocitySetpoint = null;
        _lowerVelocitySetpoint = null;

        _io.setUpperVolts(0);
        _io.setLowerVolts(0);
    }

    public void intake()
    {
        _upperVelocitySetpoint = null;
        _lowerVelocitySetpoint = null;

        _io.setUpperVolts(-_intakeVolts);
        _io.setLowerVolts(-_intakeVolts);
    }

    public void setMaxSpeed(double maxFlywheelSpeed)
    {
        _maxSpeed = maxFlywheelSpeed * Constants.General.MAX_NEO_SPEED;
    }

    public void setIntakeSpeed(double flywheelIntakeSpeed)
    {
        _intakeVolts = flywheelIntakeSpeed * Constants.General.MOTOR_VOLTAGE;
    }

    public double getUpperVelocity()
    {
        return _inputs.upperVelocity;
    }

    public double getLowerVelocity()
    {
        return _inputs.lowerVelocity;
    }

    public boolean upperAtSpeed()
    {
        return _upperVelocitySetpoint != null && Math.abs(_upperVelocitySetpoint - getUpperVelocity()) <= _upperVelocitySetpoint * _velocityRange;
    }

    public boolean lowerAtSpeed()
    {
        return _lowerVelocitySetpoint != null && Math.abs(_lowerVelocitySetpoint - getLowerVelocity()) <= _lowerVelocitySetpoint * _velocityRange;
    }

    public boolean atSpeed()
    {
        return upperAtSpeed() && lowerAtSpeed();
    }

    public boolean isShooting()
    {
        return _upperVelocitySetpoint != null && _lowerVelocitySetpoint != null && _upperVelocitySetpoint > 0 && _lowerVelocitySetpoint > 0;
    }

    public boolean isIntaking()
    {
        return _inputs.lowerVolts < 0 && _inputs.upperVolts < 0;
    }

    public void setVelocityRange(double velocityRange)
    {
        _velocityRange = velocityRange;
    }
}