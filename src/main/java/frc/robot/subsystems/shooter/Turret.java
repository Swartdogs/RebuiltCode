package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AIOConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.TurretDirector.HubTagPair;
import frc.robot.util.MeasureUtil;
import frc.robot.util.Utilities;
import limelight.Limelight;
import limelight.networktables.target.AprilTagFiducial;

@Logged
public class Turret extends SubsystemBase
{
    public enum TurretState
    {
        Idle, Track, Pass
    }

    private final TalonFX                    _turretMotor;
    private final AnalogPotentiometer        _turretSensor;
    private final Supplier<SwerveDriveState> _swerveStateSupplier;
    private final Limelight                  _limelight;
    private final PIDController              _pidController;
    private SwerveDriveState                 _currentSwerveState;
    private TurretState                      _turretState;
    @Logged
    private Angle                            _fieldAngle;
    @Logged
    private Angle                            _robotAngle;
    @Logged
    private Angle                            _turretSetpoint;
    @Logged
    private boolean                          _hasSetpoint;
    @Logged
    private Voltage                          _motorOutput;

    public Turret(Supplier<SwerveDriveState> swerveStateSupplier)
    {
        var sensorFullRange = ShooterConstants.TURRET_HARD_MAX_ANGLE.minus(ShooterConstants.TURRET_HARD_MIN_ANGLE);
        var sensorOffset    = ShooterConstants.TURRET_HARD_MIN_ANGLE;

        if (ShooterConstants.TURRET_SENSOR_INVERTED)
        {
            sensorFullRange = sensorFullRange.unaryMinus();
            sensorOffset    = sensorOffset.unaryMinus();
        }

        _swerveStateSupplier = swerveStateSupplier;

        _turretMotor   = new TalonFX(CANConstants.TURRET_MOTOR);
        _turretSensor  = new AnalogPotentiometer(AIOConstants.TURRET_POTENTIOMETER, sensorFullRange.in(Degrees), sensorOffset.in(Degrees));
        _limelight     = new Limelight(ShooterConstants.LIMELIGHT_NAME);
        _pidController = new PIDController(ShooterConstants.TURRET_KP, ShooterConstants.TURRET_KI, ShooterConstants.TURRET_KD);

        _currentSwerveState = null;
        _turretState        = TurretState.Idle;
        _turretSetpoint     = ShooterConstants.TURRET_HOME_ANGLE;
        _hasSetpoint        = false;

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT.in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.Clockwise_Positive;

        _turretMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig));

        SmartDashboard.putData("Turret PID", _pidController);
    }

    public Command stop()
    {
        return runOnce(() ->
        {
            _hasSetpoint    = false;
            _turretSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
        });
    }

    public Command setSetpoint(Angle angle)
    {
        return runOnce(() ->
        {
            _hasSetpoint    = true;
            _turretSetpoint = MeasureUtil.clamp(angle, ShooterConstants.TURRET_SOFT_MIN_ANGLE, ShooterConstants.TURRET_SOFT_MAX_ANGLE);
        });
    }

    @Override
    public void periodic()
    {
        _currentSwerveState = _swerveStateSupplier.get();

        _robotAngle = Degrees.of(_turretSensor.get());
        _fieldAngle = toFieldFrame(_robotAngle);

        _motorOutput = _turretMotor.getMotorVoltage().getValue();

        var volts    = Volts.zero();
        var tagPairs = new ArrayList<HubTagPair>();

        switch (_turretState)
        {
            case Track:
                tagPairs = getTagPairs();
            case Pass:
                _hasSetpoint = true;
                _turretSetpoint = MeasureUtil.clamp(toRobotFrame(TurretDirector.calculate(_turretState, _currentSwerveState, tagPairs)), ShooterConstants.TURRET_SOFT_MIN_ANGLE, ShooterConstants.TURRET_SOFT_MAX_ANGLE);
                break;

            case Idle:
            default:
                _hasSetpoint = false;
                _turretSetpoint = Degrees.zero();
                break;
        }

        if (_hasSetpoint)
        {
            volts = Volts.of(_pidController.calculate(_robotAngle.in(Degrees), _turretSetpoint.in(Degrees)));
        }

        // var volts = Volts.zero();

        // if (_hasSetpoint)
        // {
        // volts = Volts.of(_pidController.calculate(_robotAngle.in(Degrees),
        // _turretSetpoint.in(Degrees)));
        // }

        _turretMotor.setVoltage(volts.in(Volts));
    }

    public void setState(TurretState state)
    {
        _turretState = state;
    }

    private Angle toFieldFrame(Angle robotFrame)
    {
        return robotFrame.plus(_currentSwerveState.Pose.getRotation().getMeasure());
    }

    private Angle toRobotFrame(Angle fieldFrame)
    {
        return fieldFrame.minus(_currentSwerveState.Pose.getRotation().getMeasure());
    }

    private ArrayList<HubTagPair> getTagPairs()
    {
        ArrayList<HubTagPair> pairs = new ArrayList<HubTagPair>();

        _limelight.getLatestResults().ifPresent(results ->
        {
            // loop through fiducial targets
            // average together the angles and distances to target
            // use getTargetPose_CameraSpace2D

            // This is the leftover idk if it works
            // var tags = Collections.sort(List.of(results.targets_Fiducials), (t1, t2) ->
            // Integer.compare((int)t1.fiducialID, (int)t2.fiducialID));
            // var tagIds = Utilities.getOurHubTagIds();
        });

        return pairs;
    }
}
