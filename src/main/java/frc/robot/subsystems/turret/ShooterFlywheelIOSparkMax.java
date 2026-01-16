package frc.robot.subsystems.turret;
//TODO: Name the package for a pid controller made, then: import ;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;


public class ShooterFlywheelIOSparkMax implements ShooterFlywheelIO
{
    private final SparkMax        _motor;
    private final RelativeEncoder    _encoder;
    private void  initMotor( SparkMax motor )
    {
        // motor.initMotor();
        motor.setCANTimeout(250);
        // motor.idle(IdleMode.kCoast);
        motor.setCANTimeout( 0 ) ;

    }

    public ShooterFlywheelIOSparkMax()
    {
        _motor = new SparkMax(Constants.CAN.SHOOTERFLYWHEEL, MotorType.kBrushless ) ;
    
        initMotor( _motor ) ;
        _encoder = _motor.getEncoder() ;
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        inputs.velocity = _encoder.getVelocity();
    }

    @Override
    public void setVelocity(double velocity)
    {
        // _PID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }
}