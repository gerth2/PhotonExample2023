// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private XboxController m_controller;
  private Drivetrain m_drive;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit(){

    //At-Home Network Debug Only - host the NT server on photonvision and connect to it.
    NetworkTableInstance.getDefault().stopServer();
    NetworkTableInstance.getDefault().setServer("photonvision.local");
    NetworkTableInstance.getDefault().startClient4("MainRobotProgram");

    m_controller = new XboxController(0);
    m_drive = new Drivetrain();
  }

  @Override
  public void robotPeriodic() {
    m_drive.updateOdometry();
  }

  @Override
  public void autonomousInit() {
    m_drive.pcw.ovCam.takeInputSnapshot();
    m_drive.pcw.ovCam.takeOutputSnapshot();
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {

    //Test switching driver and pipeline
    m_drive.pcw.ovCam.setDriverMode(m_controller.getAButton());

    if(m_controller.getBButtonPressed()){
      m_drive.pcw.ovCam.setPipelineIndex(1);
    }
    if(m_controller.getBButtonReleased()){
      m_drive.pcw.ovCam.setPipelineIndex(0);
    }

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var joyY = m_controller.getLeftY();
    if(Math.abs(joyY) < 0.075){
      joyY = 0;
    }
    final var xSpeed = -m_speedLimiter.calculate(joyY) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var joyX = m_controller.getRightX();
    if(Math.abs(joyX) < 0.075){
      joyX = 0;
    }
    final var rot = -m_rotLimiter.calculate(joyX) * Drivetrain.kMaxAngularSpeed;

    m_drive.drive(xSpeed, rot);
  }

  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }
}
