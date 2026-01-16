package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class ShooterFlywheelIOSparkMax implements ShooterFlywheelIO
{
    private final CANSparkMax        _motor;
    private final CANSparkMax        _lowerMotor;
    private final RelativeEncoder    _upperEncoder;
    private final RelativeEncoder    _lowerEncoder;
    private final SparkPIDController _upperPID;
    private final SparkPIDController _lowerPID; 

    private void  initMotor( CANSparkMax motor )
    {
        motor.restoreFactoryDefaults();
        motor.setCANTimeout(250);
        motor.setSmartCurrentLimit(40);
        motor.setIdleMode(IdleMode.kCoast);
        motor.setCANTimeout( 0 ) ;
        motor.burnFlash();

    }
    private void  initPID( SparkPIDController pid )
    {
                   pid.setP(3e-5);
            pid.setI(0);
            pid.setD(0);
            pid.setIZone(0);
            pid.setFF(0.00017);
            pid.setOutputRange(-1, 1);

    }

    public ShooterFlywheelIOSparkMax()
    {
        _motor = new CANSparkMax(Constants.CAN.SHOOTER_FLYWHEEL_UPPER, MotorType.kBrushless);
        initMotor( _motor ) ;
        _encoder = _motor.getEncoder() ;
        _PID = _motor.getPIDController();
        initPID( _PID ) ;
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        inputs.velocity = _encoder.getVelocity();
    }

    @Override
    public void setVelocity(double velocity)
    {
        _PID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }
}