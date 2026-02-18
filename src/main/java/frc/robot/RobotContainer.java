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
    private final CommandXboxController          _joystick   = new CommandXboxController(0);
    private final Drive                          _drivetrain = TunerConstants.createDrivetrain();
    private final Intake                         _intake     = new Intake();
    private final Shooter                        _shooter    = new Shooter();

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
                        () -> drive.withVelocityX(MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_joystick.getLeftY())), DriveConstants.DEADBAND))// Drive forward with negative Y (forward)
                                .withVelocityY(MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_joystick.getLeftX())), DriveConstants.DEADBAND)) // Drive left with negative X (left)
                                .withRotationalRate(MeasureUtil.applyDeadband(DriveConstants.MAX_ANGULAR_RATE.times(Value.of(-_joystick.getRightX())), DriveConstants.DEADBAND)) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        _joystick.a().whileTrue(_drivetrain.applyRequest(() -> brake));
        _joystick.b().whileTrue(_drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-_joystick.getLeftY(), -_joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        _joystick.back().and(_joystick.y()).whileTrue(_drivetrain.sysIdDynamic(Direction.kForward));
        _joystick.back().and(_joystick.x()).whileTrue(_drivetrain.sysIdDynamic(Direction.kReverse));
        _joystick.start().and(_joystick.y()).whileTrue(_drivetrain.sysIdQuasistatic(Direction.kForward));
        _joystick.start().and(_joystick.x()).whileTrue(_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        _joystick.povDown().onTrue(_drivetrain.runOnce(_drivetrain::seedFieldCentric));

        _joystick.rightTrigger().onTrue(_shooter.fireCmd());
        _joystick.leftTrigger().onTrue(_shooter.stopCmd());
        _joystick.povLeft().onTrue(_shooter.startCmd(RPM.of(2000)));
        _joystick.povUp().onTrue(_shooter.startCmd(RPM.of(4000)));
        _joystick.povRight().onTrue(_shooter.startCmd(RPM.of(6000)));
        _joystick.leftBumper().onTrue(_shooter.setShootModeCmd(HoodPosition.Pass));
        _joystick.rightBumper().onTrue(_shooter.setShootModeCmd(HoodPosition.Shoot));

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
