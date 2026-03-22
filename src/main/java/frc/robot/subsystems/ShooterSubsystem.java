package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    
    private final SparkFlex shooterLeader = new SparkFlex(ShooterConstants.kShooterLeaderId, MotorType.kBrushless);
    private final SparkFlex shooterFollower = new SparkFlex(ShooterConstants.kShooterFollowerId, MotorType.kBrushless);
    private final SparkFlex feederMotor = new SparkFlex(ShooterConstants.kFeederId, MotorType.kBrushless);

    public ShooterSubsystem() {
        // Configurations
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        leaderConfig.smartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig
            .smartCurrentLimit(ShooterConstants.kShooterCurrentLimit)
            .follow(shooterLeader, true); // true = inverted relative to leader

        SparkFlexConfig feederConfig = new SparkFlexConfig();
        feederConfig.smartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

        // Apply configurations
        shooterLeader.configure(leaderConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        shooterFollower.configure(followerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        feederMotor.configure(feederConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    public Command runShooterCommand(double speed) {
        return this.run(() -> shooterLeader.set(speed))
                   .withName("Run Shooter");
    }

    public Command feedCommand(double speed) {
        return this.run(() -> feederMotor.set(speed))
                   .withName("Run Feeder");
    }

    public void stopAll() {
        shooterLeader.set(0);
        feederMotor.set(0);
    }
}