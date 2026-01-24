package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.Constants;
import frc.robot.util.GameState;
import limelight.Limelight;
import limelight.networktables.LimelightData;

public class ShooterTurret
{
    private final TalonFX         _turretMotor;
    private final PositionVoltage _positionRequest = new PositionVoltage(0).withSlot(0);
    private final Limelight       _limelight;
    private double                _angle           = 0.0;
    private Double                _angleSetpoint   = null;
    private boolean               _autoAim         = false;
    private boolean               _hasTarget       = false;

    public ShooterTurret(Limelight limelight)
    {
        _limelight = limelight;

        if (RobotBase.isSimulation())
        {
            _turretMotor = null;
            return;
        }

        _turretMotor = new TalonFX(Constants.CAN.TURRET_MOTOR);

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = Constants.Shooter.TURRET_CURRENT_LIMIT;
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.CounterClockwise_Positive;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Shooter.TURRET_KP;
        slot0Configs.kI = Constants.Shooter.TURRET_KI;
        slot0Configs.kD = Constants.Shooter.TURRET_KD;

        _turretMotor.getConfigurator().apply(currentConfig);
        _turretMotor.getConfigurator().apply(outputConfig);
        _turretMotor.getConfigurator().apply(slot0Configs);
    }

    public void periodic()
    {
        if (_turretMotor == null) return;

        _angle = _turretMotor.getPosition().getValueAsDouble() * 360.0 / Constants.Shooter.TURRET_GEAR_RATIO;

        if (_limelight != null)
        {
            _limelight.getSettings().withArilTagIdFilter(GameState.getOurHubTagIds()).save();

            LimelightData data = _limelight.getData();
            _hasTarget = data.targetData.getTargetStatus();
        }

        if (_autoAim && _limelight != null)
        {
            if (_hasTarget)
            {
                LimelightData data = _limelight.getData();
                double        tx   = data.targetData.getHorizontalOffset();
                _angleSetpoint = _angle + tx;
            }
            else
            {
                _angleSetpoint = Constants.Shooter.TURRET_HOME_ANGLE;
            }
        }

        if (_angleSetpoint != null)
        {
            double clampedSetpoint = MathUtil.clamp(_angleSetpoint, Constants.Shooter.TURRET_MIN_ANGLE, Constants.Shooter.TURRET_MAX_ANGLE);

            double targetRotations = clampedSetpoint * Constants.Shooter.TURRET_GEAR_RATIO / 360.0;
            _turretMotor.setControl(_positionRequest.withPosition(targetRotations));
        }
    }

    public void enableAutoAim()
    {
        _autoAim = true;
    }

    public void disableAutoAim()
    {
        _autoAim       = false;
        _angleSetpoint = Constants.Shooter.TURRET_HOME_ANGLE;
    }

    public void setAngle(double angleDegrees)
    {
        _angleSetpoint = angleDegrees;
        _autoAim       = false;
    }

    public void stop()
    {
        _angleSetpoint = null;
        _autoAim       = false;

        if (_turretMotor != null)
        {
            _turretMotor.setVoltage(0);
        }
    }

    public boolean atSetpoint()
    {
        if (_angleSetpoint == null) return true;
        return Math.abs(_angle - _angleSetpoint) <= Constants.Shooter.TURRET_TOLERANCE;
    }

    public boolean hasTarget()
    {
        return _hasTarget;
    }

    public boolean isAutoAiming()
    {
        return _autoAim;
    }

    public double getAngle()
    {
        return _angle;
    }

    public double getDistanceToTarget()
    {
        if (_limelight == null || !_hasTarget)
        {
            return 0.0;
        }

        var targetPose = _limelight.getData().targetData.getCameraToTarget();
        return targetPose.getTranslation().getNorm();
    }
}
