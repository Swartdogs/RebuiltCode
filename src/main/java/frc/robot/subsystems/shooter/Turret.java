package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import limelight.Limelight;

@Logged
public class Turret extends SubsystemBase 
{
    public enum TurretState
    {
        Idle, Track, Pass
    }

    private TalonFX _turretMotor; 
    private AnalogPotentiometer _turretSensor; 
    private Limelight _limelight; 
    @Logged
    private Angle _robotTurretAngle; 
    @Logged
    private Angle _fieldTurretAngle; 
    @Logged
    private Voltage _turretMotorVoltage; 
    @Logged
    private TurretState _turretState; 
    private Supplier<SwerveDriveState> _swerveStateSupplier; 
    private SwerveDriveState _swerveDriveState;

    public Turret(Supplier<SwerveDriveState> swerveStateSupplier) 
    {
        _turretMotor = new TalonFX(CANConstants.TURRET_MOTOR); 
        _turretSensor = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER); 
        _robotTurretAngle = Degrees.of(0);
        _fieldTurretAngle = Degrees.of(0);
        _turretMotorVoltage = Volts.of(0); 
        _turretState = TurretState.Idle; 
        _swerveStateSupplier = (swerveStateSupplier == null) ? () -> new SwerveDriveState() : swerveStateSupplier;
    }

    @Override
    public void periodic()
    {
        _robotTurretAngle = _turretMotor.getPosition().getValue(); 
        _swerveDriveState = _swerveStateSupplier.get(); 
        _fieldTurretAngle = _robotTurretAngle.plus(_swerveDriveState.Pose.getRotation().getMeasure()); 
        _turretMotorVoltage = _turretMotor.getMotorVoltage().getValue(); 
    }

    public void setTurretState(TurretState state) 
    {
        _turretState = state; 
    }

    public TurretState getTurretState()
    {
        return _turretState; 
    }

    public boolean isLinedUp()
    {
        boolean linedup = false; 
        switch (_turretState) {
            case Idle:
                linedup = true;
                break; 

            case Track:
                
                break;

            case Pass:

                break; 
        }

        return linedup; // TODO 
    }
}
