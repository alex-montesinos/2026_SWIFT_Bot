// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName = "limelight-NAME";

    // ==========================================
    // PHYSICAL CONSTANTS 
    // MUST measure these physically on the robot
    // ==========================================
    
    // Height from the floor to the center of your Limelight lens (Meters)
    private static final double CAMERA_HEIGHT_METERS = 0.5; 
    
    // Height from the floor to the center of the hub april tags (Meters)
    private static final double TARGET_HEIGHT_METERS = 2.0; 
    
    // The angle your camera is tilted up from perfectly horizontal (Degrees)
    private static final double CAMERA_PITCH_DEGREES = 30.0; 

    public VisionSubsystem() {}

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getTx() {
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
        
        // Average the TX if multiple tags on the hub are visible to prevent jitter
        if (fiducials.length >= 2) {
            double tx1 = fiducials[0].txnc;
            double tx2 = fiducials[1].txnc;
            return (tx1 + tx2) / 2.0; 
        } 
        else if (fiducials.length == 1) {
            return fiducials[0].txnc;
        }
        
        return 0.0; // No targets
    }

    public double getTy() {
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
        
        // Average the TY if multiple tags on the hub are visible
        if (fiducials.length >= 2) {
            double ty1 = fiducials[0].tync;
            double ty2 = fiducials[1].tync;
            return (ty1 + ty2) / 2.0; 
        } 
        else if (fiducials.length == 1) {
            return fiducials[0].tync;
        }
        
        return 0.0;
    }

    public double getDistanceToTargetMeters() {
        if (!hasTarget()) {
            return 0.0; 
        }
        
        // The vertical angle to the target from the Limelight crosshair
        double targetOffsetAngleDegrees = getTy();
        
        // Total angle from the ground
        double angleToGoalDegrees = CAMERA_PITCH_DEGREES + targetOffsetAngleDegrees;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        // Distance using tangent
        return (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / Math.tan(angleToGoalRadians);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putNumber("Vision/TX (Aiming)", getTx());
        SmartDashboard.putNumber("Vision/TY", getTy());
        SmartDashboard.putNumber("Vision/Calculated Distance (m)", getDistanceToTargetMeters());
    }
}