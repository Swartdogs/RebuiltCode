// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.MeasureUtil;

@Logged
public class RobotContainer
{
    private static final double MANUAL_FLYWHEEL_START_RPM = 3500.0;
    private static final double MANUAL_FLYWHEEL_STEP_RPM  = 50.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric _fieldCentric      = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric _robotCentric      = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry                  _logger            = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));
    private final CommandJoystick            _driver            = new CommandJoystick(0);
    private final CommandXboxController      _operator          = new CommandXboxController(1);
    private final Trigger                    _snakeMode         = _driver.button(4);
    private final Drive                      _drive             = TunerConstants.createDrivetrain();
    private final Intake                     _intake            = new Intake();
    private final Shooter                    _shooter           = new Shooter(_drive::getState);
    @NotLogged
    private final Autos                      _autos             = new Autos(_drive, _shooter);
    @NotLogged
    private final ProfiledPIDController      _snakeController   = new ProfiledPIDController(
            DriveConstants.SNAKE_HEADING_KP, 0.0, DriveConstants.SNAKE_HEADING_KD,
            new TrapezoidProfile.Constraints(DriveConstants.SNAKE_MAX_ANGULAR_RATE.in(RadiansPerSecond), DriveConstants.SNAKE_MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond))
    );
    private Dimensionless                    _driveMultiplier   = DriveConstants.FULL_SPEED_SCALE;
    private double                           _manualFlywheelRPM = MANUAL_FLYWHEEL_START_RPM;
    private Rotation2d                       _snakeHeading      = Rotation2d.kZero;
    private boolean                          _wasSnakeModeOn    = false;

    public RobotContainer()
    {
        _snakeController.enableContinuousInput(-Math.PI, Math.PI);
        _snakeController.setTolerance(DriveConstants.SNAKE_HEADING_TOLERANCE.in(Radians));
        configureBindings();
    }

    private LinearVelocity getDrive()
    {
        return MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getY())).times(_driveMultiplier), DriveConstants.TRANSLATE_DEADBAND);
    }

    private LinearVelocity getStrafe()
    {
        return MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getX())).times(_driveMultiplier), DriveConstants.TRANSLATE_DEADBAND);
    }

    private AngularVelocity getRotate()
    {
        return MeasureUtil.applyDeadband(DriveConstants.MAX_ANGULAR_RATE.times(Value.of(-_driver.getTwist())).times(_driveMultiplier), DriveConstants.ROTATE_DEADBAND);
    }

    private SwerveRequest.FieldCentric getFieldCentricRequest()
    {
        var drive   = getDrive();
        var strafe  = getStrafe();
        var rotate  = getRotate();
        var heading = _drive.getState().Pose.getRotation();
        var snakeOn = _snakeMode.getAsBoolean();

        if (snakeOn && !_wasSnakeModeOn)
        {
            resetSnakeMode(heading);
        }
        else if (!snakeOn && _wasSnakeModeOn)
        {
            _wasSnakeModeOn = false;
        }

        _wasSnakeModeOn = snakeOn;

        if (!snakeOn)
        {
            return _fieldCentric.withVelocityX(drive).withVelocityY(strafe).withRotationalRate(rotate);
        }

        return _fieldCentric.withVelocityX(drive).withVelocityY(strafe).withRotationalRate(getSnakeRotate(drive, strafe, rotate, heading));
    }

    private AngularVelocity getSnakeRotate(LinearVelocity drive, LinearVelocity strafe, AngularVelocity manualRotate, Rotation2d robotHeading)
    {
        if (Math.abs(_driver.getTwist()) > DriveConstants.SNAKE_ROTATION_OVERRIDE_AXIS_DEADBAND)
        {
            resetSnakeMode(robotHeading);
            return manualRotate;
        }

        var translationMagnitude = Math.hypot(drive.in(MetersPerSecond), strafe.in(MetersPerSecond));

        if (translationMagnitude > DriveConstants.SNAKE_MIN_TRANSLATE_FOR_HEADING.in(MetersPerSecond))
        {
            var desiredHeadingRadians = Math.atan2(strafe.in(MetersPerSecond), drive.in(MetersPerSecond)) + DriveConstants.SNAKE_INTAKE_HEADING_OFFSET.in(Radians);
            _snakeHeading = Rotation2d.fromRadians(MathUtil.angleModulus(desiredHeadingRadians));
        }

        return RadiansPerSecond.of(_snakeController.calculate(robotHeading.getRadians(), _snakeHeading.getRadians()));
    }

    private void resetSnakeMode(Rotation2d robotHeading)
    {
        _snakeHeading = robotHeading;
        _snakeController.reset(robotHeading.getRadians(), _drive.getState().Speeds.omegaRadiansPerSecond);
    }

    private void setManualFlywheelRPM(double rpm)
    {
        _manualFlywheelRPM = Math.max(0.0, rpm);
        _shooter.setManualFlywheel(_manualFlywheelRPM);
    }

    private void configureBindings()
    {
        _drive.setDefaultCommand(_drive.applyRequest(this::getFieldCentricRequest));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(_drive.applyRequest(() -> idle).ignoringDisable(true));

        _driver.button(1).whileTrue(_shooter.shoot());
        _driver.button(2).whileTrue(Commands.startEnd(() -> _driveMultiplier = DriveConstants.SLOW_MODE_SCALE, () -> _driveMultiplier = DriveConstants.FULL_SPEED_SCALE));
        _driver.button(3).whileTrue(_drive.applyRequest(() -> _robotCentric.withVelocityX(getDrive()).withVelocityY(getStrafe()).withRotationalRate(getRotate())));
        _driver.button(5).whileTrue(_shooter.pass());
        _driver.button(7).onTrue(_drive.runOnce(_drive::seedFieldCentric));

        _drive.registerTelemetry(_logger::telemeterize);

        _operator.leftTrigger().whileTrue(_intake.runRollersForward());
        _operator.leftBumper().whileTrue(_intake.runRollersReverse());
        _operator.rightTrigger().whileTrue(_intake.jiggle());
        _operator.start().and(_operator.back()).onTrue(_shooter.setManualMode(true));
        _operator.start().and(_operator.back().negate()).onTrue(_shooter.setManualMode(false));
        _operator.y().onTrue(Commands.runOnce(() -> setManualFlywheelRPM(MANUAL_FLYWHEEL_START_RPM)).onlyIf(_shooter::inManualMode));
        _operator.x().onTrue(Commands.runOnce(() -> setManualFlywheelRPM(_manualFlywheelRPM - MANUAL_FLYWHEEL_STEP_RPM)).onlyIf(_shooter::inManualMode));
        _operator.b().onTrue(Commands.runOnce(() -> setManualFlywheelRPM(_manualFlywheelRPM + MANUAL_FLYWHEEL_STEP_RPM)).onlyIf(_shooter::inManualMode));
        _operator.a().onTrue(Commands.runOnce(_shooter::stopManualFlywheel).onlyIf(_shooter::inManualMode));
        _operator.povDown().onTrue(_intake.getRetractCmd());
        _operator.povUp().onTrue(_intake.getExtendCmd());
    }

    public Command getAutonomousCommand()
    {
        return _autos.buildAuto();
    }
}
