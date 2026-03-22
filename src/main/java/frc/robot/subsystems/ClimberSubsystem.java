package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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

        climberMotor.configure(climberConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    public Command climbCommand(double speed) {
        return this.run(() -> climberMotor.set(speed))
                   .finallyDo(() -> climberMotor.set(0))
                   .withName("Run Climber");
    }
}