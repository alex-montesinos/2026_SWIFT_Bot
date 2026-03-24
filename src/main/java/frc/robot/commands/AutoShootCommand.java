package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final VisionSubsystem vision;
    
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final Timer timer = new Timer();
    
    private double currentTargetRpm = 3000.0;
    
    // Latch to prevent feeder stutter
    private boolean hasShotStarted = false;

    public AutoShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision) {
        this.shooter = shooter;
        this.intake = intake;
        this.vision = vision;

        addRequirements(shooter, intake);

        // Truth Table: (Distance in Meters, Target RPM)
        rpmMap.put(1.0, 3000.0);
        rpmMap.put(2.0, 4000.0);
        rpmMap.put(3.0, 5000.0);
        rpmMap.put(4.0, 6000.0); 
    }

    @Override
    public void initialize() {
        timer.restart();
        hasShotStarted = false; // Reset latch
        intake.setExtenderPosition(IntakeConstants.kPositionDeployed);
    }

    @Override
    public void execute() {
        // Calculate RPM
        if (vision.hasTarget()) {
            double distance = vision.getDistanceToTargetMeters();
            currentTargetRpm = rpmMap.get(distance);
        } else {
            currentTargetRpm = 3000.0; // Fallback idle speed
        }

        shooter.setTargetRpm(currentTargetRpm);

        // If it hits the speed threshold, trip latch permanently for command's cycle
        if (shooter.isAtSpeed(currentTargetRpm)) {
            hasShotStarted = true;
        }

        if (hasShotStarted) {
            shooter.setFeederSpeed(1.0); 
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
        shooter.stopAll();
        hasShotStarted = false;
        intake.setExtenderPosition(IntakeConstants.kPositionDeployed);
    }

    // Since this is used in Auto, we want it to finish eventually. 
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(5.0); 
    }
}