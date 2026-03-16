// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.LoggingConstants;

@Logged
public class Robot extends TimedRobot
{
    private Command              m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = LoggingConstants.ENABLE_CTRE_HOOT_REPLAY ? new HootAutoReplay().withTimestampReplay().withJoystickReplay() : null;

    public Robot()
    {
        if (LoggingConstants.ENABLE_WPILIB_DATA_LOG)
        {
            DataLogManager.start();
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        SignalLogger.enableAutoLogging(LoggingConstants.ENABLE_CTRE_SIGNAL_LOG);

        if (LoggingConstants.ENABLE_CTRE_SIGNAL_LOG)
        {
            SignalLogger.start();
        }
        else
        {
            SignalLogger.stop();
        }

        m_robotContainer = new RobotContainer();
        Epilogue.bind(this);
    }

    @Override
    public void robotPeriodic()
    {
        if (m_timeAndJoystickReplay != null)
        {
            m_timeAndJoystickReplay.update();
        }

        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit()
    {
    }

    @Override
    public void disabledPeriodic()
    {
    }

    @Override
    public void disabledExit()
    {
    }

    @Override
    public void autonomousInit()
    {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null)
        {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void autonomousExit()
    {
    }

    @Override
    public void teleopInit()
    {
        if (m_autonomousCommand != null)
        {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void teleopExit()
    {
    }

    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic()
    {
    }

    @Override
    public void testExit()
    {
    }

    @Override
    public void simulationInit()
    {
        RoboRioSim.setVInVoltage(GeneralConstants.MOTOR_VOLTAGE.in(Volts));
        RoboRioSim.setUserVoltage5V(GeneralConstants.SENSOR_VOLTAGE.in(Volts));
    }

    @Override
    public void simulationPeriodic()
    {
    }
}
