/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package statespacesimulation;

import edu.wpi.first.hal.sim.RoboRioSim;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.sim.SimBattery;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is a sample program to demonstrate the use of state-space classes in robot simulation.
 * This robot has a flywheel, elevator, arm and differential drivetrain, and interfaces with
 * the sim GUI's {@link edu.wpi.first.wpilibj.simulation.Field2d} class.
 */
public class Robot extends TimedRobot {

  private RoboRioSim m_roborioSim;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Our simulated RoboRio. Used to simulate battery voltages.
    if (isSimulation()) {
      m_roborioSim = new RoboRioSim(0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // Here we calculate the battery voltage based on drawn current.
    // As our robot draws more power from the battery its voltage drops.
    // The estimated voltage is highly dependent on the battery's internal
    // resistance.
    var drawCurrent = m_robotContainer.getRobotDrive().getDrawnCurrentAmps();
    var loadedVoltage = SimBattery.calculateLoadedBatteryVoltage(drawCurrent);
    m_roborioSim.setVInVoltage(loadedVoltage);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
