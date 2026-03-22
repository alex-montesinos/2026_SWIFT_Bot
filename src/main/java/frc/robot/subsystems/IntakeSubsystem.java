package frc.robot.subsystems;

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

    public IntakeSubsystem() {
        // Extender Config (SPARK MAX)
        SparkMaxConfig extenderConfig = new SparkMaxConfig();
        extenderConfig.smartCurrentLimit(IntakeConstants.kCurrentLimitAmps);
        extenderMotor.configure(extenderConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        // Roller Config (SPARK Flex)
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.smartCurrentLimit(IntakeConstants.kCurrentLimitAmps);
        rollerMotor.configure(rollerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    public Command runRollerCommand(double speed) {
        return this.run(() -> rollerMotor.set(speed))
                   .withName("Run Intake Roller");
    }

    public Command extendCommand(double speed) {
         return this.run(() -> extenderMotor.set(speed))
                    .withName("Move Extender");
    }
}