// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.rotor.Rotor;
import frc.robot.util.MeasureUtil;

@Logged
public class RobotContainer
{
    private final SwerveRequest.FieldCentric _fieldCentric = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric _robotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final CommandJoystick            _driver       = new CommandJoystick(0);
    private final CommandXboxController      _operator     = new CommandXboxController(1);
    private final Drive                      _drive        = TunerConstants.createDrivetrain();
    private final Intake                     _intake       = new Intake();
    private final Autos                      _autos        = new Autos(_drive, _intake);
    // private Dimensionless _driveMultiplier = DriveConstants.FULL_SPEED_SCALE;

    // Shooter subsystem components
    private final Rotor _rotor = new Rotor();

    public RobotContainer()
    {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private LinearVelocity getDrive()
    {
        return MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getY())).times((-_driver.getRawAxis(3) + 1) * 0.375 + 0.25), DriveConstants.TRANSLATE_DEADBAND);
    }

    private LinearVelocity getStrafe()
    {
        return MeasureUtil.applyDeadband(DriveConstants.MAX_SPEED.times(Value.of(-_driver.getX())).times((-_driver.getRawAxis(3) + 1) * 0.375 + 0.25), DriveConstants.TRANSLATE_DEADBAND);
    }

    private AngularVelocity getRotate()
    {
        return MeasureUtil.applyDeadband(DriveConstants.MAX_ANGULAR_RATE.times(Value.of(-_driver.getTwist())).times((-_driver.getRawAxis(3) + 1) * 0.375 + 0.25), DriveConstants.ROTATE_DEADBAND);
    }

    private SwerveRequest.FieldCentric getFieldCentricRequest()
    {
        return _fieldCentric.withVelocityX(getDrive()).withVelocityY(getStrafe()).withRotationalRate(getRotate());
    }

    private void configureDefaultCommands()
    {
        _drive.setDefaultCommand(_drive.applyRequest(this::getFieldCentricRequest));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(_drive.applyRequest(() -> idle).ignoringDisable(true));
    }

    private void configureButtonBindings()
    {
        _driver.button(2).whileTrue(_intake.runRollersForward());
        _driver.button(3).onTrue(_intake.getExtendCmd());
        _driver.button(4).onTrue(_intake.getRetractCmd());
        _driver.button(7).onTrue(_drive.runOnce(_drive::seedFieldCentric));
        _driver.button(16).whileTrue(_drive.applyRequest(() -> _robotCentric.withVelocityX(getDrive()).withVelocityY(getStrafe()).withRotationalRate(getRotate())));

        _operator.leftTrigger().whileTrue(_intake.runRollersForward());
        _operator.leftBumper().whileTrue(_intake.runRollersReverse());
        _operator.rightTrigger().whileTrue(_intake.jiggle());
        _operator.povDown().onTrue(_intake.getRetractCmd());
        _operator.povUp().onTrue(_intake.getExtendCmd());

        // Rotor testing commands
        _operator.a().onTrue(_rotor.commands.stop());
        _operator.y().onTrue(_rotor.commands.setRate(RPM.of(80)));
        _operator.b().whileTrue(_rotor.commands.run(RPM.of(100)));
        _operator.x().whileTrue(_rotor.commands.run(RPM.of(60)));
    }

    public Command getAutonomousCommand()
    {
        return _autos.buildAuto();
    }
}
