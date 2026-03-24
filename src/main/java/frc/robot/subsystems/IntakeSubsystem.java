package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final SparkMax extenderMotor = new SparkMax(IntakeConstants.kExtenderId, MotorType.kBrushless);
    private final SparkFlex rollerMotor = new SparkFlex(IntakeConstants.kRollerId, MotorType.kBrushless);

    private boolean isDeployed = true; // Tracking toggle state
    private double currentTargetPosition = IntakeConstants.kPositionDeployed;

    public IntakeSubsystem() {
        SparkMaxConfig extenderConfig = new SparkMaxConfig();
        
        extenderConfig.smartCurrentLimit(IntakeConstants.kCurrentLimitAmps)
                      .idleMode(IdleMode.kBrake); // Hold position against gravity/impacts
        
        extenderConfig.closedLoop
            .pid(IntakeConstants.kExtenderP, IntakeConstants.kExtenderI, IntakeConstants.kExtenderD, ClosedLoopSlot.kSlot0);
            
        extenderConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(IntakeConstants.kPositionDeployed + 1.0) // slightly past deploy target
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(IntakeConstants.kPositionRetracted - 0.5); // slightly past retract target

        // --- ROLLER CONFIG ---
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.smartCurrentLimit(IntakeConstants.kCurrentLimitAmps)
                    .idleMode(IdleMode.kCoast);

        // Apply configurations
        extenderMotor.configure(extenderConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        rollerMotor.configure(rollerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        // Reset encoder to 0 on startup (Requires starting the match completely retracted!)
        extenderMotor.getEncoder().setPosition(0);
    }

    // --- ROLLER METHODS ---
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public Command runRollerCommand(double speed) {
        return this.runEnd(
            () -> setRollerSpeed(speed),
            () -> setRollerSpeed(0)
        ).withName("Run Intake Roller");
    }

    // --- EXTENDER METHODS ---
    public void setExtenderPosition(double targetRotations) {
        currentTargetPosition = targetRotations;
        
        extenderMotor.getClosedLoopController().setSetpoint(
            targetRotations, 
            ControlType.kPosition, 
            ClosedLoopSlot.kSlot0
        );
    }

    public Command setPositionCommand(double targetRotations) {
        return this.run(() -> setExtenderPosition(targetRotations))
                   .withName("Set Extender Position");
    }

    public Command toggleDeploymentCommand() {
        return this.runOnce(() -> {
            isDeployed = !isDeployed;
            double target = isDeployed ? IntakeConstants.kPositionDeployed : IntakeConstants.kPositionRetracted;
            setExtenderPosition(target);
        }).withName("Toggle Intake");
    }

    @Override
    public void periodic() {
        // Telemetry for PID tuning
        SmartDashboard.putNumber("Intake/Target Position", currentTargetPosition);
        SmartDashboard.putNumber("Intake/Actual Position", extenderMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake/Roller Current (A)", rollerMotor.getOutputCurrent());
    }
}