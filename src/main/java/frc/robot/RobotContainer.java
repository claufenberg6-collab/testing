// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Flywheel1;
import frc.robot.commands.Flywheel2;
import frc.robot.commands.pivotdown;
import frc.robot.commands.pivotup;
import frc.robot.commands.pivotup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.flywheel1;
import frc.robot.subsystems.flywheel2;
import frc.robot.subsystems.pivot;
import frc.robot.subsystems.pivot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

public final DriveSubsystem m_robotDrive = new DriveSubsystem();
private final SlewRateLimiter xLimiter = new SlewRateLimiter(8.0);
private final SlewRateLimiter yLimiter = new SlewRateLimiter(8.0);
private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    private final pivot m_pivot = new pivot();
    private final flywheel1 m_flywheel1 = new flywheel1();
    private final flywheel2 m_flywheel2 = new flywheel2();



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
       private final CommandXboxController m_codriverController =
     new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {













    double x = -MathUtil.applyDeadband(
m_driverController.getLeftY(), OIConstants.kDriveDeadband);

double y = -MathUtil.applyDeadband(
m_driverController.getLeftX(), OIConstants.kDriveDeadband);

double rot = -MathUtil.applyDeadband(
m_driverController.getRightX(), OIConstants.kDriveDeadband);


 x = xLimiter.calculate(x);
 y = yLimiter.calculate(y);
 rot = rotLimiter.calculate(rot);

 m_robotDrive.drive(x, y, rot, true,true);

  m_robotDrive.setDefaultCommand(
     
       new RunCommand(
           () ->
               m_robotDrive.drive(
                   -MathUtil.applyDeadband(
                       m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                   -MathUtil.applyDeadband(
                       m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                   -MathUtil.applyDeadband(
                       m_driverController.getRightX(), OIConstants.kDriveDeadband),
                   true,true),
           m_robotDrive).withName("Robot Drive Default"));
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {  
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());
    m_driverController.b().whileTrue(new pivotup(m_pivot));
    m_driverController.a().whileTrue(new pivotdown(m_pivot));
    m_driverController.a().toggleOnTrue(new Flywheel1(m_flywheel1));
    m_driverController.a().toggleOnTrue(new Flywheel2(m_flywheel2));
  }
  }

