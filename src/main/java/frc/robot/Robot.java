// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Telemetry;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;



// import au.grapplerobotics.LaserCan;
// import au.grapplerobotics.ConfigurationFailedException;
 
public class Robot extends TimedRobot {
  // private LaserCan lc;
  
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private double value1 = 0.0;
  private double lastUpdateTime = 0.0;
  private final double UPDATE_PERIOD = 0.1;
  private DoubleLogEntry txLog;
  private DoubleLogEntry poseXLog;
  private DoubleLogEntry poseYLog;
  private DoubleLogEntry poseRotationLog;
  private int value = 0;


  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    DataLogManager.start(); 
    DataLogManager.log("Started");
    txLog = new DoubleLogEntry(DataLogManager.getLog(), "Limelight/tx");
    poseXLog = new DoubleLogEntry(DataLogManager.getLog(), "Limelight/botpose_orb_wpiblue/X");
    poseYLog = new DoubleLogEntry(DataLogManager.getLog(), "Limelight/botpose_orb_wpiblue/Y");
    poseRotationLog = new DoubleLogEntry(DataLogManager.getLog(), "Limelight/botpose_orb_wpiblue/Rotation");

    


    // lc = new LaserCan(0);
    // // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    // try {
    //   lc.setRangingMode(LaserCan.RangingMode.SHORT);
    //   lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    //   lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    // } catch (ConfigurationFailedException e) {
    //   System.out.println("Configuration failed! " + e);
    // }
    


  }

  @Override
  public void robotPeriodic() {   
    
    // LaserCan.Measurement measurement = lc.getMeasurement();
    // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
    //   System.out.println("The target is " + measurement.distance_mm + "mm away!");
    // } else {
    //   System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
    //   // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    // } 
    
    double tx = LimelightHelpers.getTX("limelight");
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    if (poseEstimate != null) {
        poseXLog.append(poseEstimate.pose.getX());
        poseYLog.append(poseEstimate.pose.getY());
        poseRotationLog.append(poseEstimate.pose.getRotation().getDegrees());
        
        // Optional: Log the full pose as a string
       //DataLogManager.log("Limelight botpose_orb_wpiblue: " + poseEstimate.pose.toString());
}

    // Replace this with your actual PoseEstimate
  
        
    
    
    txLog.append(tx); 
    
 if (Timer.getFPGATimestamp() - lastUpdateTime < UPDATE_PERIOD) return;
        lastUpdateTime = Timer.getFPGATimestamp();

        // Get pose estimate from Limelight
        LimelightHelpers.PoseEstimate PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-threeg");
        Pose2d pose1 = PoseEstimate.pose;
        value++;

        // Check if data is valid
        boolean hasTarget = LimelightHelpers.getLatestResults("limelight-threeg").valid;
        int tagCount = PoseEstimate.tagCount;
        double avgTagDist = PoseEstimate.avgTagDist;

        // Push to SmartDashboard
        SmartDashboard.putNumber("Pose_X", pose1.getX());
        SmartDashboard.putNumber("Pose_Y", pose1.getY());
        SmartDashboard.putNumber("Pose_Rotation_Deg", pose1.getRotation().getDegrees());
        SmartDashboard.putNumber("Counter", value1);
        SmartDashboard.putBoolean("Has_Target", hasTarget);
        SmartDashboard.putNumber("Tag_Count", tagCount);
        SmartDashboard.putNumber("Avg_Tag_Distance", avgTagDist);

        // Debug to console for raw values
        if (tagCount > 0) {
            System.out.println("Pose X: " + pose1.getX() + ", Y: " + pose1.getY() + ", Rot: " + pose1.getRotation().getDegrees());
        } else {
            System.out.println("No tags detected!");
        }
    }
  
  /*  DataLogManager.log("Limelight botpose_orb_wpiblue" + LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-threeg"));
    CommandScheduler.getInstance().run(); 
    LimelightHelpers object = new LimelightHelpers();
    LimelightHelpers.PoseEstimate pose = object.getBotPoseEstimate_wpiBlue("limelight-threeg");
    Pose2d pose1 = poseEstimate.pose;


    value++;
    SmartDashboard.putNumber("Estimated X: ", pose1.getX());
    SmartDashboard.putNumber("Estimated Y: ", pose1.getY());
    SmartDashboard.putNumber("Number: ", value);

    SmartDashboard.updateValues(); 
  } */

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
