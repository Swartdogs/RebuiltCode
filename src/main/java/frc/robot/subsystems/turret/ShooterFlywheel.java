package frc.robot.subsystems.turret;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.Flywheel;
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
   // private final ShooterFlywheelIOInputsAutoLogged _inputs                = new ShooterFlywheelIOInputsAutoLogged();
    private double                                  _maxSpeed              = Constants.Flywheel.MAX_FLYWHEEL_SPEED * Constants.General.MAX_NEO_SPEED;
    private double                                  _velocityRange         = Constants.Flywheel.VELOCITY_RANGE;
    private Double                                  _upperVelocitySetpoint = null;
    private Double                                  _lowerVelocitySetpoint = null;
    private final PIDController _flyvolPID;

    public ShooterFlywheel(ShooterFlywheelIO flywheelIO)
    {
        _io = flywheelIO;
        _flyvolPID = new PIDController(0.03, 0, 0);
    }
     private void  initPID( PIDController pid )
    {
        
        pid.setTolerance(Flywheel.PIDTolerance);
       //      pid.setIZone(0);
         //   pid.setFF(0.00017);
           // pid.setOutputRange(-1, 1);'''
         
        initPID( _flyvolPID ) ;

    }
    @Override
    public void periodic()
    {
        // _io.updateInputs(_inputs);
        // Logger.processInputs("Shooter/Flywheel", _inputs);
    }

    public void setUpperVelocity(double upperVelocity)
    {
        _upperVelocitySetpoint = MathUtil.clamp(upperVelocity, 0, _maxSpeed);
        Logger.recordOutput("Shooter/Flywheel/UpperVelocitySetpoint", _upperVelocitySetpoint);
        _io.setVelocity(_upperVelocitySetpoint);
    }

    public void stop()
    {
        _io.stop() ;
    }

    public void setMaxSpeed(double maxFlywheelSpeed)
    {
        _maxSpeed = maxFlywheelSpeed * Constants.General.MAX_NEO_SPEED;
    }

    public boolean atSpeed()
    {
        return _upperVelocitySetpoint != null && Math.abs(_upperVelocitySetpoint - _io.getVelocity()) <= _upperVelocitySetpoint * _velocityRange;
    }

    public boolean isShooting()
    {
        return _upperVelocitySetpoint != null && _lowerVelocitySetpoint != null && _upperVelocitySetpoint > 0 && _lowerVelocitySetpoint > 0;
    }

    public void setVelocityRange(double velocityRange)
    {
        _velocityRange = velocityRange;
    }
}