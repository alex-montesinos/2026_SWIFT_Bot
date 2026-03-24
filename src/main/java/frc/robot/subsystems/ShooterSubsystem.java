package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex leaderMotor;
    private final SparkFlex followerMotor;
    private final SparkFlex feederMotor;

    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;

    private double targetRpm = 0;
    
    // Speed margin of error
    private final double RPM_TOLERANCE = 150; 
    
    // Requires the RPM to be within tolerance for 0.05 seconds before reporting "true"
    private final Debouncer atSpeedDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kRising);

    // Feedforward: Calculates base voltage needed to hold an RPM
    // Tune these (kS, kV, kA) using the SysId tool for true accuracy
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0.0015, 0);

    public ShooterSubsystem() {
        leaderMotor = new SparkFlex(22, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        followerMotor = new SparkFlex(21, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkFlex(23, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        closedLoopController = leaderMotor.getClosedLoopController();
        encoder = leaderMotor.getEncoder();

        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        SparkFlexConfig feederConfig = new SparkFlexConfig();

        leaderConfig.idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(60)
                    .closedLoopRampRate(0.25); // Takes 0.25s to reach max output (helps battery)

        // Configure PID to correct for RPM
        leaderConfig.closedLoop
                    .pid(0.0005, 0.0, 0.0, ClosedLoopSlot.kSlot0);

        followerConfig.idleMode(IdleMode.kCoast)
                      .smartCurrentLimit(60)
                      .follow(leaderMotor, true);
        
        feederConfig.idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40); 

        // Apply configurations
        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    public void setTargetRpm(double rpm) {
        this.targetRpm = rpm;
        
        if (rpm <= 0) {
            leaderMotor.set(0); 
        } else {
            double ffVoltage = feedforward.calculate(rpm);
            
            closedLoopController.setSetpoint(
                rpm, 
                ControlType.kVelocity, 
                ClosedLoopSlot.kSlot0, 
                ffVoltage, 
                ArbFFUnits.kVoltage
            );
        }
    }

    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }

    /**
     * Checks if the shooter is at the target RPM
     */
    public boolean isAtSpeed(double expectedRpm) {
        if (expectedRpm == 0) return false;
        
        boolean isCurrentlyInTolerance = Math.abs(encoder.getVelocity() - expectedRpm) <= RPM_TOLERANCE;
        
        return atSpeedDebouncer.calculate(isCurrentlyInTolerance);
    }

    // Auto/Base Commands
    public Command runShooterCommand(double rpm) {
        return this.run(() -> setTargetRpm(rpm)).withName("Run Shooter");
    }

    public Command feedCommand(double speed) {
        return this.run(() -> setFeederSpeed(speed)).withName("Run Feeder");
    }

    public void stopAll() {
        setTargetRpm(0);
        setFeederSpeed(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Target RPM", targetRpm);
        SmartDashboard.putNumber("Shooter/Current RPM", encoder.getVelocity());
        SmartDashboard.putBoolean("Shooter/Is At Speed", isAtSpeed(targetRpm));
    }
}