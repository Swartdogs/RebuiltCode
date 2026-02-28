package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.MotorHook;
import frc.robot.TestHook;

@Logged
public class Flywheel
{
    private static final String             kSettingsPrefix  = "Dashboard/Dashboard Settings/";
    private final SparkFlex                 _leadMotor;
    private final SparkFlex                 _followMotor;
    private final SparkClosedLoopController _closedLoopController;
    private final FlywheelSim               _flywheelSim;
    private final RelativeEncoder           _flywheelEncoder;
    private final SparkFlexSim              _leadMotorSim;
    @Logged
    private AngularVelocity                 _velocity        = RPM.zero();
    @Logged
    private AngularVelocity                 _targetVelocity  = RPM.zero();
    @Logged
    private Voltage                         _flywheelVoltage = Volts.zero();
    private double                          _lastKP;
    private double                          _lastKD;
    private double                          _lastKS;
    private double                          _lastKV;
    private double                          _lastKA;

    public Flywheel()
    {
        _leadMotor   = new SparkFlex(CANConstants.FLYWHEEL_LEAD, MotorType.kBrushless);
        _followMotor = new SparkFlex(CANConstants.FLYWHEEL_FOLLOW, MotorType.kBrushless);

        if (RobotBase.isReal())
        {
            _leadMotor.setCANTimeout(250);
            _followMotor.setCANTimeout(250);
        }

        var config = new SparkFlexConfig();
        config.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit((int)ShooterConstants.FLYWHEEL_CURRENT_LIMIT.in(Amps)).voltageCompensation(GeneralConstants.MOTOR_VOLTAGE.in(Volts));
        config.closedLoop.p(ShooterConstants.FLYWHEEL_KP).d(ShooterConstants.FLYWHEEL_KD);
        config.closedLoop.feedForward.kS(ShooterConstants.FLYWHEEL_KS.in(Volts)).kV(ShooterConstants.FLYWHEEL_KV.in(Volts.per(RPM))).kA(ShooterConstants.FLYWHEEL_KA.in(Volts.per(RotationsPerSecondPerSecond)));
        _leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(_leadMotor, true);
        _followMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _closedLoopController = _leadMotor.getClosedLoopController();
        _flywheelEncoder      = _leadMotor.getEncoder();

        _lastKP = ShooterConstants.FLYWHEEL_KP;
        _lastKD = ShooterConstants.FLYWHEEL_KD;
        _lastKS = ShooterConstants.FLYWHEEL_KS.in(Volts);
        _lastKV = ShooterConstants.FLYWHEEL_KV.in(Volts.per(RPM));
        _lastKA = ShooterConstants.FLYWHEEL_KA.in(Volts.per(RotationsPerSecondPerSecond));

        initializeTuningPreferences();

        if (RobotBase.isReal())
        {
            _leadMotor.setCANTimeout(0);
            _followMotor.setCANTimeout(0);
            _leadMotorSim = null;
            _flywheelSim  = null;
        }
        else
        {
            var gearbox = DCMotor.getNeoVortex(2);
            _leadMotorSim = new SparkFlexSim(_leadMotor, DCMotor.getNeoVortex(2));
            _flywheelSim  = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, 0.001, 1), gearbox);
        }
    }

    public void periodic()
    {
        _velocity        = RPM.of(_flywheelEncoder.getVelocity());
        _flywheelVoltage = Volts.of(_leadMotor.getAppliedOutput() * _leadMotor.getBusVoltage());

        double newP = Preferences.getDouble(kSettingsPrefix + "Flywheel kP", ShooterConstants.FLYWHEEL_KP);
        double newD = Preferences.getDouble(kSettingsPrefix + "Flywheel kD", ShooterConstants.FLYWHEEL_KD);
        double newS = Preferences.getDouble(kSettingsPrefix + "Flywheel kS", ShooterConstants.FLYWHEEL_KS.in(Volts));
        double newV = Preferences.getDouble(kSettingsPrefix + "Flywheel kV", ShooterConstants.FLYWHEEL_KV.in(Volts.per(RPM)));
        double newA = Preferences.getDouble(kSettingsPrefix + "Flywheel kA", ShooterConstants.FLYWHEEL_KA.in(Volts.per(RotationsPerSecondPerSecond)));
        if (newP != _lastKP || newD != _lastKD || newS != _lastKS || newV != _lastKV || newA != _lastKA)
        {
            var config = new SparkFlexConfig();
            config.closedLoop.p(newP).d(newD);
            config.closedLoop.feedForward.kS(newS).kV(newV).kA(newA);
            _leadMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            _lastKP = newP;
            _lastKD = newD;
            _lastKS = newS;
            _lastKV = newV;
            _lastKA = newA;
        }
    }

    public void simulationPeriodic()
    {
        _flywheelSim.setInputVoltage(_flywheelVoltage.in(Volts));
        _flywheelSim.update(GeneralConstants.LOOP_PERIOD.in(Seconds));
        var velocity = _flywheelSim.getAngularVelocity().in(RPM);
        _leadMotorSim.getRelativeEncoderSim().setVelocity(velocity);
        _leadMotorSim.iterate(velocity, RoboRioSim.getVInVoltage(), GeneralConstants.LOOP_PERIOD.in(Seconds));
    }

    public void setVelocity(AngularVelocity targetVelocity)
    {
        if (targetVelocity == null)
        {
            stop();
            return;
        }
        _targetVelocity = targetVelocity;
        _closedLoopController.setSetpoint(_targetVelocity.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void stop()
    {
        _targetVelocity = RPM.zero();
        _leadMotor.setVoltage(0.0);
    }

    public boolean atSpeed()
    {
        if (_targetVelocity == RPM.zero()) return true;
        return getVelocity().isNear(_targetVelocity, ShooterConstants.FLYWHEEL_TOLERANCE.in(Value));
    }

    public AngularVelocity getVelocity()
    {
        return _velocity;
    }

    private static void initPreference(String key, double value)
    {
        Preferences.initDouble(kSettingsPrefix + key, value);
    }

    private static void initializeTuningPreferences()
    {
        initPreference("Flywheel kP", ShooterConstants.FLYWHEEL_KP);
        initPreference("Flywheel kD", ShooterConstants.FLYWHEEL_KD);
        initPreference("Flywheel kS", ShooterConstants.FLYWHEEL_KS.in(Volts));
        initPreference("Flywheel kV", ShooterConstants.FLYWHEEL_KV.in(Volts.per(RPM)));
        initPreference("Flywheel kA", ShooterConstants.FLYWHEEL_KA.in(Volts.per(RotationsPerSecondPerSecond)));
        initPreference("Flywheel Tolerance", ShooterConstants.FLYWHEEL_TOLERANCE.in(Value));
        initPreference("Flywheel Pass Velocity", ShooterConstants.PASS_FLYWHEEL_VELOCITY.in(RPM));
    }

    private class FlywheelHook extends MotorHook
    {
        @Override
        public void stop()
        {
            _leadMotor.stopMotor();
        }

        @Override
        public void setRate(double rate)
        {
            _leadMotor.setVoltage(GeneralConstants.MOTOR_VOLTAGE.times(rate * _polarity));
        }
    }

    public TestHook getHook()
    {
        return new FlywheelHook();
    }
}
