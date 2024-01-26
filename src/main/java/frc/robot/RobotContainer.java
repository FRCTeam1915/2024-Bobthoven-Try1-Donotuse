// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Swerve.OperatorConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_IntakeController = new CommandXboxController(OperatorConstants.kIntakeControllerPort);

    /* Drive Controls */
    private final int translationAxis = (int) m_driverController.getLeftY();
    private final int strafeAxis = (int) m_driverController.getLeftX();
    private final int rotationAxis = (int) m_driverController.getRightX();

    /* Driver Buttons */
    
    //Code for using joysticks
    private final JoystickButton zeroGyro =
            new JoystickButton(driver, XboxController.Button.kY.value);
   // private final XboxController robotCentric =
//       new XboxController(m_driverController, XboxController.Button.kLeftBumper.value);
    
    //private final XboxController zeroGyro = new XboxController(m_driverController, XboxController.Button.kY.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -m_driverController.getRawAxis(translationAxis),
                        () -> -m_driverController.getRawAxis(strafeAxis),
                        () -> -m_driverController.getRawAxis(rotationAxis),
                        () -> m_driverController.x().getAsBoolean()));

                        

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */


        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        
        
        //zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    Trigger abutton = m_driverController.a();
    

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new ExampleAuto(s_Swerve);
    }
}
