// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Swerve.OperatorConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    //private final Joystick driver = new Joystick(0);


    public final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_IntakeController = new CommandXboxController(OperatorConstants.kIntakeControllerPort);
    
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    public int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    
    //Code for using joysticks
    //private final XboxController robotCentric = new XboxController(XboxController.Button.kLeftBumper.value);
    
    /* Subsystems */
    public final static Swerve s_Swerve = new Swerve();

    
    public static SendableChooser<Double> autonomousPosition = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        

    //UsbCamera camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(Constants.CAMERA_RES_X, Constants.CAMERA_RES_Y);

        s_Swerve.setDefaultCommand(

                new TeleopSwerve(
                        s_Swerve,
                        () -> -m_driverController.getRawAxis(translationAxis),
                        () -> -m_driverController.getRawAxis(strafeAxis),
                        () -> -m_driverController.getRawAxis(rotationAxis),
                        () -> m_driverController.x().getAsBoolean()));
                        

        // Configure the button bindings
        configureButtonBindings();

        SmartDashboard.putData("Autonomous Setting", autonomousPosition);
        autonomousPosition.addOption("Position 1 - Left Side", 10.0);
        autonomousPosition.setDefaultOption("Position 2 - Middle", 7.5);
        autonomousPosition.addOption("Position 3 - Right Side", 10.0);
    }   


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        Trigger driver_a_button = m_driverController.a();
        driver_a_button.whileTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        

        // final Intake upperMotor;
        // final Intake lowerMotor;
        // upperMotor = new Intake();
        // lowerMotor = new Intake();

        // final Intake shooterMotorOne;
        // shooterMotorOne = new Intake();
        // final Intake shooterMotorTwo;
        // shooterMotorTwo = new Intake();

        // final Intake armMotorOne;
        // final Intake armMotorTwo;
        // armMotorOne = new Intake();
        // armMotorTwo = new Intake();

        Trigger xButton = m_IntakeController.x();
        xButton.whileTrue(new upperIntake().repeatedly());
    
        Trigger rightBumper = m_IntakeController.rightBumper();
        rightBumper.whileTrue(new pickUp(true).repeatedly());

        Trigger rightTrigger = m_IntakeController.rightTrigger();
        rightTrigger.whileTrue(new lowerIntakeDrop(true).repeatedly());

        Trigger AButton = m_IntakeController.a();
        AButton.whileTrue(new shooter(.8, 0));

        Trigger Ybutton = m_IntakeController.y();
        Ybutton.whileTrue(new shooter(.2, 0));

        Trigger Bbutton = m_IntakeController.b();
        Bbutton.whileTrue(new shooter(-.1, 0));

        Trigger povDown = m_IntakeController.povDown();
        povDown.whileTrue(new intakeArm(-.1));

        Trigger povUp = m_IntakeController.povUp();
        povUp.whileTrue(new intakeArm(.1));

        Trigger povLeft = m_IntakeController.povLeft();
        povLeft.whileTrue(new intakeArm(.025));

        Trigger povRight = m_IntakeController.povRight();
        povRight.whileTrue(new intakeArm(-.025));

        Trigger getAButton = m_driverController.a();
        getAButton.whileTrue(new driveToNote());
    

        




    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */


    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new ExampleAuto(s_Swerve);
    }
}
