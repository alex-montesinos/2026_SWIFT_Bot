package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private final SparkMax climberMotor = new SparkMax(ClimberConstants.kClimberId, MotorType.kBrushless);

    public ClimberSubsystem() {
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        
        climberConfig
            .smartCurrentLimit(ClimberConstants.kClimberCurrentLimit)
            .idleMode(IdleMode.kBrake);

        // ==========================================
        // Soft Limits
        // MUST physically measure these rotations
        // ==========================================
        climberConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(ClimberConstants.kMaxHeightRotations) // Maximum safe extension height
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0); // Fully retracted state

        climberMotor.configure(climberConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        // Climber fully retracted
        climberMotor.getEncoder().setPosition(0);
    }

    public Command climbCommand(double speed) {
        return this.run(() -> climberMotor.set(speed))
                   .finallyDo(() -> climberMotor.set(0))
                   .withName("Run Climber");
    }

    /**
     * Manually zero the climber encoder.
     */
    public Command zeroEncoderCommand() {
        return this.runOnce(() -> climberMotor.getEncoder().setPosition(0))
                   .ignoringDisable(true)
                   .withName("Zero Climber Encoder");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Position (Rots)", climberMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Climber/Current (A)", climberMotor.getOutputCurrent());
        
        SmartDashboard.putBoolean("Climber/Stall Warning", climberMotor.getOutputCurrent() > 35.0);
    }
}