package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final SparkMax extenderMotor = new SparkMax(IntakeConstants.kExtenderId, MotorType.kBrushless);
    private final SparkFlex rollerMotor = new SparkFlex(IntakeConstants.kRollerId, MotorType.kBrushless);

    private boolean isDeployed = true; // Tracking state for the toggle

    public IntakeSubsystem() {
        // Extender Config
        SparkMaxConfig extenderConfig = new SparkMaxConfig();
        extenderConfig.smartCurrentLimit(IntakeConstants.kCurrentLimitAmps);
        
        // Configure the PID controller
        extenderConfig.closedLoop
            .pid(IntakeConstants.kExtenderP, IntakeConstants.kExtenderI, IntakeConstants.kExtenderD, ClosedLoopSlot.kSlot0);
            
        // Reset encoder to 0 on startup
        extenderMotor.getEncoder().setPosition(0);
        extenderMotor.configure(extenderConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        // Roller Config
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.smartCurrentLimit(IntakeConstants.kCurrentLimitAmps);
        rollerMotor.configure(rollerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    // Roller Methods
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public Command runRollerCommand(double speed) {
        return this.runEnd(
            () -> setRollerSpeed(speed),
            () -> setRollerSpeed(0)
        ).withName("Run Intake Roller");
    }

    // Extender Methods
    public void setExtenderPosition(double targetRotations) {
        extenderMotor.getClosedLoopController().setSetpoint(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
}