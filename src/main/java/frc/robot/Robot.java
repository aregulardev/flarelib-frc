// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.flarerobotics.lib.BatteryUpdater;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    @SuppressWarnings("unused")
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        BatteryUpdater.setNominalVoltage(12.5);;

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        BatteryUpdater.update();
        System.out.println("Voltage: " + RoboRioSim.getVInVoltage());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // m_robotContainer.sim.setSetpoint(State.TEST);
        // m_robotContainer.sim.setRPM(State.TEST);
    }

    @Override
    public void teleopPeriodic() {
        /*
         * System.out.println(
         * Timer.getFPGATimestamp() + "::" +
         * m_robotContainer.sim.getCurrentHeight_Unit().in(Centimeters));
         */
       //  System.out.println(
       //         Timer.getFPGATimestamp() + "::" + m_robotContainer.sim.getRPM());
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
