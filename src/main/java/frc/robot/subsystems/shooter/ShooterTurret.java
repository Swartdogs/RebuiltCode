package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Utilities;
import frc.robot.util.GameState;
import limelight.Limelight;
import limelight.networktables.LimelightData;

@Logged
public class ShooterTurret
{
    public enum TurretState
    {
        Off, Track, Pass
    }

    private final TalonFX         _turretMotor;
    private final PositionVoltage _positionRequest        = new PositionVoltage(0).withSlot(0);
    private final Limelight       _limelight;
    private final TalonFXSimState _turretMotorSim;
    private final DCMotorSim      _motorSimModel;
    @Logged

    private TurretState           _state                  = TurretState.Off;
    @Logged
    private double                _angle                  = 0.0;
    private Double                _angleSetpoint   = null;
    @Logged
    private double                _targetHorizontalOffset = 0.0;
    @Logged
    private boolean               _hasTarget              = false;
    @Logged
    private double                _angleSetpoint          = Double.NaN;
    private boolean               _autoAim         = false;

    public ShooterTurret()
    {
        _turretMotor = new TalonFX(CANConstants.TURRET_MOTOR);

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit       = ShooterConstants.TURRET_CURRENT_LIMIT;
        currentConfig.StatorCurrentLimitEnable = true;

        var outputConfig = new MotorOutputConfigs();
        outputConfig.NeutralMode = NeutralModeValue.Brake;
        outputConfig.Inverted    = InvertedValue.CounterClockwise_Positive;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = ShooterConstants.TURRET_KP;
        slot0Configs.kI = ShooterConstants.TURRET_KI;
        slot0Configs.kD = ShooterConstants.TURRET_KD;

        _turretMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentConfig).withMotorOutput(outputConfig).withSlot0(slot0Configs));

        _limelight = new Limelight(ShooterConstants.LIMELIGHT_NAME);

        if (RobotBase.isReal())
        {
            _turretMotorSim = null;
            _motorSimModel  = null;
        }
        else
        {
            _turretMotorSim = _turretMotor.getSimState();

            var gearbox = DCMotor.getKrakenX44Foc(1);
            _motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, ShooterConstants.TURRET_GEAR_RATIO), gearbox);
        }
    }

    public void periodic()
    {
        if (_turretMotor == null) return;

        // TODO: Consider configuring Phoenix 6 sensor/gear ratio so the reported
        // position is in mechanism degrees.
        _angle = _turretMotor.getPosition().getValueAsDouble() * 360.0 / ShooterConstants.TURRET_GEAR_RATIO;

        if (_limelight != null)
        {
            _limelight.getSettings().withArilTagIdFilter(Utilities.getOurHubTagIds()).save();

            LimelightData data = _limelight.getData();
            _hasTarget              = data.targetData.getTargetStatus();
            _targetHorizontalOffset = data.targetData.getHorizontalOffset();
        }

        switch (_state)
        {
            case Off:
                _angleSetpoint = Double.NaN;
                break;

            case Track:
                if (_hasTarget)
                {
                    _angleSetpoint = _angle + _targetHorizontalOffset;
                }
                else
                {
                    _angleSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
                }
                break;

            case Pass:
                // TODO: Implement pass angle logic
                _angleSetpoint = ShooterConstants.TURRET_HOME_ANGLE;
                break;
        }

        if (!Double.isNaN(_angleSetpoint))
        {
            double clampedSetpoint = MathUtil.clamp(_angleSetpoint, ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE);
            double targetRotations = clampedSetpoint * ShooterConstants.TURRET_GEAR_RATIO / 360.0;
            _turretMotor.setControl(_positionRequest.withPosition(targetRotations));
        }
        
        if (_autoAim && _limelight != null)
        {
            _angle = _turretMotor.getPosition().getValueAsDouble() * 360.0 / Constants.Shooter.TURRET_GEAR_RATIO;

            _limelight.getSettings().withArilTagIdFilter(GameState.getOurHubTagIds()).save();

            LimelightData data = _limelight.getData();
            _hasTarget = data.targetData.getTargetStatus();
            _tx        = data.targetData.getHorizontalOffset();

            switch (_state)
            {
                case Off:
                    _angleSetpoint = null;
                    break;

                case Track:
                    if (_hasTarget)
                    {
                        _angleSetpoint = _angle + _tx;
                    }
                    else
                    {
                        _angleSetpoint = Constants.Shooter.TURRET_HOME_ANGLE;
                    }
                    break;

                case Pass:
                    // TODO: Implement pass angle logic
                    _angleSetpoint = Constants.Shooter.TURRET_HOME_ANGLE;
                    break;
            }
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

    public void simulationPeriodic()
    {
        _turretMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

        var motorVoltage = _turretMotorSim.getMotorVoltageMeasure();
        _motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        _motorSimModel.update(GeneralConstants.LOOP_PERIOD_SECS);

        _turretMotorSim.setRawRotorPosition(_motorSimModel.getAngularPosition().times(ShooterConstants.TURRET_GEAR_RATIO));
        _turretMotorSim.setRotorVelocity(_motorSimModel.getAngularVelocity().times(ShooterConstants.TURRET_GEAR_RATIO));
    }

    public void setState(TurretState state)
    {
        _state = state;
    }

    public TurretState getState()
    {
        return _state;
    }

    public boolean atSetpoint()
    {
        if (_angleSetpoint == null) return true;
        return Math.abs(_angle - _angleSetpoint) <= ShooterConstants.TURRET_TOLERANCE;
    }

    public boolean hasTarget()
    {
        return _hasTarget;
    }

    public double getAngle()
    {
        return _angle;
    }

    public double getTargetHorizontalOffset()
    {
        return _targetHorizontalOffset;
    }

    public double getTX()
    {
        return _targetHorizontalOffset;
    }

    public double getDistanceToTarget()
    {
        if (!_hasTarget)
        {
            return 0.0;
        }

        var targetPose = _limelight.getData().targetData.getCameraToTarget();
        return targetPose.getTranslation().toTranslation2d().getNorm();
    }
}
