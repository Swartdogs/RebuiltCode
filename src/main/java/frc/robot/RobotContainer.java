// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.TestOperation;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Hood.HoodPosition;
import frc.robot.util.MeasureUtil;

@Logged
public class RobotContainer
{

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric     drive       = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake       = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt    point       = new SwerveRequest.PointWheelsAt();
    private final Telemetry                      _logger     = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));
    private final CommandXboxController          _driver     = new CommandXboxController(0);
    private final CommandXboxController          _operator   = new CommandXboxController(1);
    private final Drive                          _drivetrain = TunerConstants.createDrivetrain();
    private final Intake                         _intake     = new Intake();
    private final Shooter                        _shooter    = new Shooter(_drivetrain::getState);
    private final TestOperation                  _testop     = new TestOperation();
    // private final Turret _turret = new Turret(_drivetrain::getState);
    private final Autos _autos = new Autos(_drivetrain);

    public RobotContainer()
    {
        configureBindings();
    }

    private void configureBindings()
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        _drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                _drivetrain.applyRequest(
                        () -> drive.withVelocityX(MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getLeftY())), DriveConstants.DEADBAND))// Drive forward with negative Y (forward)
                                .withVelocityY(MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getLeftX())), DriveConstants.DEADBAND)) // Drive left with negative X (left)
                                .withRotationalRate(MeasureUtil.applyDeadband(DriveConstants.MAX_ANGULAR_RATE.times(Value.of(-_driver.getRightX())), DriveConstants.DEADBAND)) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        _driver.a().whileTrue(_drivetrain.applyRequest(() -> brake));
        _driver.b().whileTrue(_drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-_driver.getLeftY(), -_driver.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        _driver.x().whileTrue(_drivetrain.sysIdDynamic(Direction.kForward));
        _driver.y().whileTrue(_drivetrain.sysIdDynamic(Direction.kReverse));
        _driver.leftBumper().whileTrue(_drivetrain.sysIdQuasistatic(Direction.kForward));
        _driver.rightBumper().whileTrue(_drivetrain.sysIdQuasistatic(Direction.kReverse));

        _operator.a().onTrue(Commands.runOnce(() -> _intake.extend(!_intake.isExtended()), _intake));
        _operator.x().onTrue(_intake.startRollers());
        _operator.y().onTrue(_intake.reverseRollers());
        _operator.leftStick().onTrue(_intake.stopRollers());

        _operator.rightBumper().onTrue(_shooter.setShootModeCmd(HoodPosition.Shoot));
        _operator.leftBumper().onTrue(_shooter.passCmd());
        _operator.povUp().onTrue(_shooter.startCmd(RPM.of(4500)));
        _operator.povLeft().onTrue(_shooter.startCmd(RPM.of(3500)));
        _operator.leftTrigger().onTrue(_shooter.stopCmd());
        _operator.rightTrigger().onTrue(_shooter.fireCmd());
        // Reset the field-centric heading on left bumper press.
        _driver.povDown().onTrue(_drivetrain.runOnce(_drivetrain::seedFieldCentric));

        _drivetrain.registerTelemetry(_logger::telemeterize);

    }

    private void configureTestBindings()
    {
        _testop.add("intake", _intake.getHook());
        _testop.add("intake-extend", _intake.getHookExt());
        _testop.add("feeder", _shooter._feeder.getHook());
        _testop.add("flywheel", _shooter._flywheel.getHook());
        _testop.add("hood", _shooter._hood.getHook());
        _testop.add("turret", _shooter._turret.getHook());
        // climber

        _testop.connect(0, "intake", "intake-extend");
        _testop.connect(1, "feeder");
        _testop.connect(2, "turret");
        _testop.connect(3, "flywheel", "hood");

        _operator.leftBumper().whileTrue(_testop.cmd_shift());
        _operator.a().onTrue(_testop.cmd_button_04());
        _operator.b().onTrue(_testop.cmd_button_15());
        _operator.x().onTrue(_testop.cmd_button_26());
        _operator.y().onTrue(_testop.cmd_button_37());
        _operator.leftTrigger().whileTrue(_testop.cmd_reverse());
        _operator.rightTrigger().whileTrue(_testop.cmd_forward());
        _operator.povUp().onTrue(_testop.cmd_increase());
        _operator.povDown().onTrue(_testop.cmd_decrease());
        _operator.povLeft().onTrue(_testop.cmd_jog());
        _operator.povRight().onTrue(_testop.cmd_full());
        _operator.start().onTrue(_testop.cmd_reset());
    }

    public Command getAutonomousCommand()
    {
        return _autos.followPath("LeftTrenchToDepot");
    }
}
