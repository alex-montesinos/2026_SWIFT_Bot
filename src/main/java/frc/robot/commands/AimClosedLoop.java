package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimClosedLoop extends Command {
    private final DriveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final VisionSubsystem vision;
    
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final PIDController aimPID = new PIDController(0.04, 0.0, 0.005); 
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    
    private final Timer timer = new Timer();

    public AimClosedLoop(DriveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, 
                         DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.vision = vision;
        this.translationXSupplier = xSupplier;
        this.translationYSupplier = ySupplier;

        addRequirements(swerve, shooter, intake);

        // Truth Table: (Distance in Meters, Target RPM)
        rpmMap.put(1.0, 3000.0);
        rpmMap.put(2.0, 4000.0);
        rpmMap.put(3.0, 5000.0);
        rpmMap.put(4.0, 6000.0); 
    }

    @Override
    public void initialize() {
        timer.restart();
        intake.setExtenderPosition(IntakeConstants.kPositionDeployed);
    }

    @Override
    public void execute() {
        double translationX = -MathUtil.applyDeadband(translationXSupplier.getAsDouble(), OIConstants.kDriveDeadband);
        double translationY = -MathUtil.applyDeadband(translationYSupplier.getAsDouble(), OIConstants.kDriveDeadband);
        
        double rotationSpeed = 0.0;
        double targetRpm = 3000.0; // Idle/Default RPM

        if (vision.hasTarget()) {
            rotationSpeed = aimPID.calculate(vision.getTx(), 0.0);
            double distance = vision.getDistanceToTargetMeters();
            targetRpm = rpmMap.get(distance);
        }

        // Aim Robot
        swerve.drive(translationX, translationY, rotationSpeed, true); 

        // Rev the shooter
        shooter.setTargetRpm(targetRpm);

        // Feeder Logic
        if (shooter.isAtSpeed(targetRpm)) {
            shooter.setFeederSpeed(1.0); // Run feeder at full speed
        } else {
            shooter.setFeederSpeed(0.0);
        }

        // Intake Agitation (After 3 seconds)
        if (timer.hasElapsed(3.0)) {
            if (timer.get() % 0.6 > 0.3) {
                intake.setExtenderPosition(IntakeConstants.kPositionAgitate);
            } else {
                intake.setExtenderPosition(IntakeConstants.kPositionDeployed);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.setTargetRpm(0);
        shooter.setFeederSpeed(0.0);
        intake.setExtenderPosition(IntakeConstants.kPositionDeployed);
    }
}