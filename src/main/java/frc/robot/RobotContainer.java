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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.TestOperation;
// import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.util.MeasureUtil;

@Logged
public class RobotContainer
{

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric     drive       = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake       = new SwerveRequest.SwerveDriveBrake();
    private final Telemetry                      _logger     = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));
    private final CommandJoystick                _driver     = new CommandJoystick(0);
    private final CommandXboxController          _operator   = new CommandXboxController(1);
    private final Drive                          _drivetrain = TunerConstants.createDrivetrain();
    private final Intake                         _intake     = new Intake();
    private final Shooter                        _shooter    = new Shooter(_drivetrain::getState);
    private final TestOperation                  _testop     = new TestOperation();
    private final Autos                          _autos      = new Autos(_drivetrain);
    private final Turret                         _turret     = new Turret(_drivetrain::getState);
    // private final Dashboard _dashboard = new Dashboard(_intake, _shooter,
    // _shooter._turret);

    public RobotContainer()
    {
        configureBindings();
        // configureTestBindings();
    }

    private void configureBindings()
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        _drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                _drivetrain.applyRequest(
                        () -> drive.withVelocityX(MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getY())), DriveConstants.TRANSLATE_DEADBAND))
                                .withVelocityY(MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getX())), DriveConstants.TRANSLATE_DEADBAND))
                                .withRotationalRate(MeasureUtil.applyDeadband(DriveConstants.MAX_ANGULAR_RATE.times(Value.of(-_driver.getTwist())), DriveConstants.ROTATE_DEADBAND))
                )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(_drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        _turret.setTurretState(Turret.TurretState.Idle);

        _driver.button(1).whileTrue(_shooter.smartShootCmd(_turret::getHubDistance, () -> _shooter._flywheel.atSpeed() && _turret.isLinedUp()));
        _driver.button(2).whileTrue(_drivetrain.applyRequest(() -> brake));
        _driver.button(7).onTrue(_drivetrain.runOnce(_drivetrain::seedFieldCentric));

        _drivetrain.registerTelemetry(_logger::telemeterize);

        _operator.leftTrigger().whileTrue(_intake.startRollers());
        _operator.leftBumper().whileTrue(_intake.reverseRollers());
        _operator.povDown().onTrue(_intake.getExtendCmd());
        _operator.povUp().onTrue(_intake.getRetractCmd());

        _operator.y().onTrue(_shooter.setVelocity(RPM.of(3500)));
        _operator.b().onTrue(_shooter.modVelocity(RPM.of(200)));
        _operator.a().onTrue(_shooter.modVelocity(RPM.of(-200)));
        _operator.x().onTrue(_shooter.stopCmd());

        _operator.rightTrigger().whileTrue(_shooter.runFeeder());
    }

    private void configureTestBindings()
    {
        _testop.add("intake", _intake.getHook());
        _testop.add("intake-extend", _intake.getHookExt());
        _testop.add("feeder", _shooter._feeder.getHook());
        _testop.add("flywheel", _shooter._flywheel.getHook());
        _testop.add("hood", _shooter._hood.getHook());
        // _testop.add("turret", _shooter._turret.getHook());
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
