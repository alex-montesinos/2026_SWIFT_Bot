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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex leaderMotor;
    private final SparkFlex followerMotor;
    private final SparkFlex feederMotor;

    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;

    private double targetRpm = 0;
    private final double RPM_TOLERANCE = 100; // Speed margin of error

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0.0015, 0);

    public ShooterSubsystem() {
        // Assuming IDs based on your previous messages
        leaderMotor = new SparkFlex(22, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        followerMotor = new SparkFlex(21, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkFlex(23, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        closedLoopController = leaderMotor.getClosedLoopController();
        encoder = leaderMotor.getEncoder();

        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        SparkFlexConfig feederConfig = new SparkFlexConfig();

        leaderConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);   
        
        followerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60).follow(leaderMotor, true);
        
        feederConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40); // Brake mode prevents coasting shots

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

    public boolean isAtSpeed(double expectedRpm) {
        if (expectedRpm == 0) return false;
        // Checks if current velocity is within the defined tolerance
        return Math.abs(encoder.getVelocity() - expectedRpm) <= RPM_TOLERANCE;
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
}