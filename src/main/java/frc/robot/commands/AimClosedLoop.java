package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimClosedLoop extends Command {
    private final DriveSubsystem swerve;
    private final VisionSubsystem vision;
    
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final PIDController aimPID = new PIDController(0.04, 0.0, 0.005); 

    public AimClosedLoop(DriveSubsystem swerve, VisionSubsystem vision, 
                         DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.swerve = swerve;
        this.vision = vision;
        this.translationXSupplier = xSupplier;
        this.translationYSupplier = ySupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double translationX = -MathUtil.applyDeadband(translationXSupplier.getAsDouble(), OIConstants.kDriveDeadband);
        double translationY = -MathUtil.applyDeadband(translationYSupplier.getAsDouble(), OIConstants.kDriveDeadband);
        
        double rotationSpeed = 0.0;

        if (vision.hasTarget()) {
            rotationSpeed = aimPID.calculate(vision.getTx(), 0.0);
        }

        swerve.drive(translationX, translationY, rotationSpeed, true); 
    }
}