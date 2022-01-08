// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class RomiCamera extends SubsystemBase {
  

  // How far from the target we want to be
  // final double GOAL_RANGE_METERS = Units.feetToMeters(0.25);

  // Change this to match the name of your camera
  PhotonCamera m_camera = new PhotonCamera("mmal_service_16.1");
  private PhotonPipelineResult m_result;
  
  // Moving average filter used to smooth out target range
  private MedianFilter m_filter = new MedianFilter(10);
  private double m_range;
  private double m_lastRange = 0.0;
  private double m_lastPitch = 0.0;

  private Pose2d m_position = new Pose2d();

  // Moving average filter used to check for a consistent target
  private MedianFilter m_targetFilter = new MedianFilter(10);
  private LinearFilter m_pitchFilter = LinearFilter.singlePoleIIR(1.0, 0.02);
  private int m_gotTarget = 0;

  // PIDController m_rangeController = new PIDController(2.6, 0.0, 2.0);

  /** 
   * Constructor
   * Creates a new RomiCamera that gets its data from PhotonVision
   * */
  public RomiCamera() {
    m_result = m_camera.getLatestResult();
  }

  /**
   * Gets the observed yaw of the target.  
   * 
   * @return The latest yaw of the target from the camera in degrees
   */
  public double getYaw() {
    // Gets the latestResult and determines if a target is aquired
    if (hasTargets()) {
      // Tuning - Reduce the yaw value.
      return m_result.getBestTarget().getYaw() * 0.5;
    } else {
      System.out.println("NO Target!!!" );
      return 0.0;
    } 
  }

  public double getYawRadians() {
    return Units.degreesToRadians(getYaw());
  }

  /**
   * Gets the observed pitch of the target.  
   * 
   * @return The latest pitch of the target from the camera in degrees
   */
  public double getPitch() {
    if (hasTargets()) {
      m_lastPitch = m_result.getBestTarget().getPitch();
    }
    return m_pitchFilter.calculate(m_lastPitch);
  }

  public double getPitchRadians() {
    return Units.degreesToRadians(getPitch());
  }

  /**
   * Calculates how far the camera is from the target
   * 
   * @return range in meters from the target
   */
  public double getRange() {
    if (hasTargets()) {
      m_range = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAMERA_HEIGHT_METERS,
                VisionConstants.TARGET_HEIGHT_METERS,
                VisionConstants.CAMERA_PITCH_RADIANS,
                getPitchRadians());

      m_lastRange = m_range;          
      
    } 
    // Always return the last range and remove outliers
    double filteredRange = m_filter.calculate(m_lastRange);

    // Tune range to compensate for inaccuracies in calculation.  
    // This can happen if there's very little difference between
    // camera height and target height
    filteredRange = filteredRange * 2.5;

    if (filteredRange < 0.9) {
      filteredRange = filteredRange * 0.7;
    }
    
    if (filteredRange > 1.2) {
      filteredRange = filteredRange * 1.1;
    }
    // return -m_rangeController.calculate(filteredRange);
    return filteredRange;
  }

  /**
    * Estimate the Translation2d of the target relative to the camera.
    */
  public Translation2d getCameraToTargetTranslation() {
    return PhotonUtils.estimateCameraToTargetTranslation(getRange(), 
                                                         new Rotation2d(getYawRadians()));
  }

  /**
    * Estimates a Transform2d that maps the camera position to the target position, 
    * using the robot's gyro. 
    *
    * @param gyroAngle A Rotation2d of the gyro angle in radians
    */
  public Transform2d getCameraToTarget(Rotation2d gyroAngle) {
    return PhotonUtils.estimateCameraToTarget(getCameraToTargetTranslation(),
                                              // A Pose2d representing the target position in the field coordinate system.
                                              VisionConstants.kFieldToEndTarget,
                                              // The current robot gyro angle, likely from odometry.
                                              gyroAngle);
  }

  /**
    * Estimate the position of the robot on the field.
    *
    * @param gyroAngle The gyro angle in degrees
    *
    * @return The Pose2d position of the robot on the field.
    */
  public Pose2d getFieldToRobot(double gyroAngle) {
    if (hasTargets()) {
      // The current robot gyro angle, likely from odometry.
      Rotation2d gyroRotation = new Rotation2d(Units.degreesToRadians(gyroAngle));

      // Transform2d cameraToTarget, Pose2d fieldToTarget, Transform2d cameraToRobot
      m_position = PhotonUtils.estimateFieldToRobot(getCameraToTarget(gyroRotation), 
                                                    VisionConstants.kFieldToEndTarget, 
                                                    VisionConstants.kCameraToRobot);
    }
    return m_position;
  }
  
  public double getSkew() {
    return m_result.getBestTarget().getSkew();
  }

  /**
   * Gets the latestResult and determines if a target is aquired
   * 
   * @return True if target is observed, false otherwise
   */
  public boolean hasTargets() {
    m_result = m_camera.getLatestResult();
    return m_result.hasTargets();
  }

  public boolean lostTarget() {
    if (hasTargets()) {
      m_gotTarget = 1;
    } else {
      m_gotTarget = 0;
    }
    // If we got a target in the last 10 cycles return true
    double result = m_targetFilter.calculate(m_gotTarget);
    if (result > 0) {
      return false;
    } else {
      return true;
    }
  }

  public double getTargetHeight() {
    return VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.CAMERA_HEIGHT_METERS;
  }

  public double getCameraPitch() {
    return VisionConstants.CAMERA_PITCH_RADIANS;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_result = m_camera.getLatestResult();
  }
}
