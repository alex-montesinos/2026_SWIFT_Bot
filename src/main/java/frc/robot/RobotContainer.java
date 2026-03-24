// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimClosedLoop;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
  
  // --- Subsystems ---
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem(m_robotDrive);

  // --- Controllers ---
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  // --- Auto Chooser ---
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // PATHPLANNER NAMED COMMANDS
    NamedCommands.registerCommand("Run Intake", m_intake.runRollerCommand(Constants.IntakeConstants.kIntakeSpeed));
    NamedCommands.registerCommand("Deploy Intake", m_intake.setPositionCommand(Constants.IntakeConstants.kPositionDeployed));
    NamedCommands.registerCommand("Retract Intake", m_intake.setPositionCommand(Constants.IntakeConstants.kPositionRetracted));
    NamedCommands.registerCommand("Stop Shooter", Commands.runOnce(m_shooter::stopAll, m_shooter));
    NamedCommands.registerCommand("Auto Shoot", new AutoShootCommand(m_shooter, m_intake, m_vision));
    NamedCommands.registerCommand("Auto Aim", new AimClosedLoop(m_robotDrive, m_vision, null, null));

    configureButtonBindings();
    configureMatchDataTriggers();

    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Mode", autoChooser);

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    // --- DRIVER CONTROLS ---
    m_driverController.rightBumper()
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    m_driverController.start()
        .onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // --- OPERATOR CONTROLS ---
    m_operatorController.leftTrigger()
        .whileTrue(m_intake.runRollerCommand(Constants.IntakeConstants.kIntakeSpeed));

    m_operatorController.y().onTrue(m_intake.toggleDeploymentCommand());

    // SHOOT & AIM (Right Trigger)
    // Runs the aiming drivetrain command AND the shooting command simultaneously
        m_operatorController.rightTrigger().whileTrue(
            Commands.parallel(
                new AimClosedLoop(
                    m_robotDrive, 
                    m_vision,
                    () -> m_driverController.getLeftY(),
                    () -> m_driverController.getLeftX()
                ),
                new AutoShootCommand(m_shooter, m_intake, m_vision)
            )
        );

    m_operatorController.povUp()
        .whileTrue(m_climber.climbCommand(Constants.ClimberConstants.kClimbUpSpeed));
    m_operatorController.povDown()
        .whileTrue(m_climber.climbCommand(Constants.ClimberConstants.kClimbDownSpeed));
  }

  // MATCH DATA & RUMBLE LOGIC
  private void configureMatchDataTriggers() {
    new Trigger(this::isHubActive)
        .onTrue(getRumbleCommand(1.0)); 

    new Trigger(this::isHubActive)
        .onFalse(getRumbleCommand(1.5)); 
  }

  public boolean isHubActive() {
      if (!DriverStation.isTeleopEnabled()) return false;
      double time = DriverStation.getMatchTime();
      
      if (time > 130.0 || time <= 30.0) return true;

      String gameData = DriverStation.getGameSpecificMessage();
      
      if (gameData == null || gameData.isEmpty()) return true;

      var alliance = DriverStation.getAlliance();
      boolean isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
      
      boolean ourGoalInactiveFirst = (isRedAlliance && gameData.charAt(0) == 'R') || 
                                     (!isRedAlliance && gameData.charAt(0) == 'B');

      boolean isShift1 = time <= 130.0 && time > 105.0;
      boolean isShift2 = time <= 105.0 && time > 80.0;
      boolean isShift3 = time <= 80.0 && time > 55.0;
      boolean isShift4 = time <= 55.0 && time > 30.0;

      if (ourGoalInactiveFirst) {
          return isShift2 || isShift4;
      } else {
          return isShift1 || isShift3;
      }
  }

  public String getMatchStateString() {
      if (DriverStation.isAutonomousEnabled()) return "AUTO";
      if (DriverStation.isTeleopEnabled()) {
          double t = DriverStation.getMatchTime();
          if (t > 130.0) return "TRANSITION SHIFT";  
          if (t > 105.0) return "SHIFT 1";           
          if (t > 80.0)  return "SHIFT 2";           
          if (t > 55.0)  return "SHIFT 3";           
          if (t > 30.0)  return "SHIFT 4";           
          return "END GAME";                         
      }
      return "DISABLED";
  }

  private Command getRumbleCommand(double seconds) {
    return Commands.run(() -> {
        m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
    }).withTimeout(seconds)
    .finallyDo(() -> {
        m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }).withName("Controller Rumble Alert");
  }

  public void updateDashboard() {
      double time = Math.max(0, DriverStation.getMatchTime()); 
      
      int minutes = (int) (time / 60);
      int seconds = (int) (time % 60);
      String timeString = String.format("%d:%02d", minutes, seconds);

      SmartDashboard.putString("Match/Time Remaining", timeString);
      SmartDashboard.putString("Match/Current Shift", getMatchStateString());
      SmartDashboard.putBoolean("Match/HUB ACTIVE", isHubActive());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}