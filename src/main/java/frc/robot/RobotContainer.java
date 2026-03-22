// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // --- Subsystems ---
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // --- Controllers ---
  // Using CommandXboxController instead of XboxController for cleaner bindings
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // ==========================================
    // DRIVER CONTROLS
    // ==========================================
    
    // Right Bumper -> Set wheels to X-formation to lock defensive position
    m_driverController.rightBumper()
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Start Button -> Zero the gyro heading
    m_driverController.start()
        .onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // ==========================================
    // OPERATOR CONTROLS
    // ==========================================

    // INTAKE: Right Trigger to extend and run roller, retract and stop when released
    m_operatorController.rightTrigger()
        .whileTrue(m_intake.extendCommand(Constants.IntakeConstants.kExtenderSpeed)
            .alongWith(m_intake.runRollerCommand(Constants.IntakeConstants.kIntakeSpeed)))
        .onFalse(m_intake.extendCommand(-Constants.IntakeConstants.kExtenderSpeed).withTimeout(0.5));

    // SHOOTER: Right Bumper to spin up Vortex motors
    m_operatorController.rightBumper()
        .whileTrue(m_shooter.runShooterCommand(Constants.ShooterConstants.kShooterTargetSpeed));

    // FEEDER: Left Bumper to feed the FUEL into the spun-up shooter
    m_operatorController.leftBumper()
        .whileTrue(m_shooter.feedCommand(Constants.ShooterConstants.kFeederSpeed));

    // CLIMBER: D-Pad Up to climb, D-Pad Down to lower
    m_operatorController.povUp()
        .whileTrue(m_climber.climbCommand(Constants.ClimberConstants.kClimbUpSpeed));
    m_operatorController.povDown()
        .whileTrue(m_climber.climbCommand(Constants.ClimberConstants.kClimbDownSpeed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Simple Auto: Shoot preloaded then back up.
    return Commands.sequence(
        // 1. Spin up the shooter to target speed for 1 second
        m_shooter.runShooterCommand(Constants.ShooterConstants.kShooterTargetSpeed)
                 .withTimeout(1.0),
        
        // 2. Keep the shooter running and run the feeder to shoot the preload for 10 seconds
        Commands.parallel(
            m_shooter.runShooterCommand(Constants.ShooterConstants.kShooterTargetSpeed),
            m_shooter.feedCommand(Constants.ShooterConstants.kFeederSpeed)
        ).withTimeout(10),

        // 3. Stop the shooter mechanisms
        Commands.runOnce(m_shooter::stopAll, m_shooter),

        // 4. Drive backward at 20% speed for 2 seconds to cross the mobility line
        // drive(xSpeed, ySpeed, rot, fieldRelative)
        Commands.run(() -> m_robotDrive.drive(-0.2, 0, 0, false), m_robotDrive)
                .withTimeout(2.0)
                .andThen(() -> m_robotDrive.drive(0, 0, 0, false))
    );
  }
}