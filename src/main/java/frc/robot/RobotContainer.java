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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;
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
    private final CommandJoystick                _driver     = new CommandJoystick(0);
    private final CommandXboxController          _operator   = new CommandXboxController(1);
    private final Drive                          _drivetrain = TunerConstants.createDrivetrain();
    private final Intake                         _intake     = new Intake();
    private final Shooter                        _shooter    = new Shooter();
    private final Turret                         _turret     = new Turret(_drivetrain::getState);

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
                        () -> drive.withVelocityX(MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getY())), DriveConstants.DEADBAND))// Drive forward with negative Y (forward)
                                .withVelocityY(MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getX())), DriveConstants.DEADBAND)) // Drive left with negative X (left)
                                .withRotationalRate(MeasureUtil.applyDeadband(DriveConstants.MAX_ANGULAR_RATE.times(Value.of(-_driver.getTwist())), DriveConstants.DEADBAND)) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        _driver.button(1).whileTrue(_drivetrain.applyRequest(() -> brake));
        _driver.button(2).whileTrue(_drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-_driver.getY(), -_driver.getX()))));

        //Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
         _driver.button(3).whileTrue(_drivetrain.sysIdDynamic(Direction.kForward));
         _driver.button(4).whileTrue(_drivetrain.sysIdDynamic(Direction.kReverse));
         _driver.button(5).whileTrue(_drivetrain.sysIdQuasistatic(Direction.kForward));
         _driver.button(6).whileTrue(_drivetrain.sysIdQuasistatic(Direction.kReverse));

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

    public Command getAutonomousCommand()
    {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(

                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                _drivetrain.runOnce(() -> _drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                _drivetrain.applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0)).withTimeout(5.0),
                // Finally idle for the rest of auton
                _drivetrain.applyRequest(() -> idle)
        );
    }
}
