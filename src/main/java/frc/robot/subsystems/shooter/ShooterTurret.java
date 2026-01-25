package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

import frc.robot.Constants;
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
    private final PositionVoltage _positionRequest = new PositionVoltage(0).withSlot(0);
    private final Limelight       _limelight;
    private final TalonFXSimState _turretMotorSim;
    private final DCMotorSim      _motorSimModel;
    @Logged
    private TurretState           _state           = TurretState.Off;
    @Logged
    private double                _angle           = 0.0;
    @Logged
    private double                _tx              = 0.0;
    @Logged
    private boolean               _hasTarget       = false;
    private Double                _angleSetpoint   = null;

    public ShooterTurret()
    {
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

        _limelight = new Limelight(Constants.Shooter.LIMELIGHT_NAME);

        if (RobotBase.isReal())
        {
            _turretMotorSim = null;
            _motorSimModel  = null;
        }
        else
        {
            _turretMotorSim = _turretMotor.getSimState();

            var gearbox = DCMotor.getKrakenX60Foc(1);
            _motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, Constants.Shooter.TURRET_GEAR_RATIO), gearbox);
        }
    }

    public void periodic()
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

        if (_angleSetpoint != null)
        {
            double clampedSetpoint = MathUtil.clamp(_angleSetpoint, Constants.Shooter.TURRET_MIN_ANGLE, Constants.Shooter.TURRET_MAX_ANGLE);
            double targetRotations = clampedSetpoint * Constants.Shooter.TURRET_GEAR_RATIO / 360.0;
            _turretMotor.setControl(_positionRequest.withPosition(targetRotations));
        }
        else
        {
            _turretMotor.setVoltage(0);
        }
    }

    public void simulationPeriodic()
    {
        _turretMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

        var motorVoltage = _turretMotorSim.getMotorVoltageMeasure();
        _motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        _motorSimModel.update(Constants.General.LOOP_PERIOD_SECS);

        _turretMotorSim.setRawRotorPosition(_motorSimModel.getAngularPosition().times(Constants.Shooter.TURRET_GEAR_RATIO));
        _turretMotorSim.setRotorVelocity(_motorSimModel.getAngularVelocity().times(Constants.Shooter.TURRET_GEAR_RATIO));
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
        return Math.abs(_angle - _angleSetpoint) <= Constants.Shooter.TURRET_TOLERANCE;
    }

    public boolean hasTarget()
    {
        return _hasTarget;
    }

    public double getAngle()
    {
        return _angle;
    }

    public double getTX()
    {
        return _tx;
    }

    public double getDistanceToTarget()
    {
        if (!_hasTarget)
        {
            return 0.0;
        }

        var targetPose = _limelight.getData().targetData.getCameraToTarget();
        return targetPose.getTranslation().getNorm();
    }
}
