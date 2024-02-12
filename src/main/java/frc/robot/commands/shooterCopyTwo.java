// package frc.robot.commands;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkLowLevel;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.Intake;

// public class shooterCopyTwo extends CommandBase{
//     public static CANSparkFlex shooterMotorOne;
//     public static CANSparkFlex shooterMotorTwo;
//     // Intake shooterMotorOne;
//     // Intake shooterMotorTwo;
//     private double in;
//     public shooterCopyTwo(double in) {
//         // in = tin;
//        // addRequirements(upperMotor);
//     //    shooterMotorOne = new CANSparkFlex(Constants.shooterOneID, CANSparkLowLevel.MotorType.kBrushless);
//     //    shooterMotorTwo = new CANSparkFlex(Constants.shooterTwoID, CANSparkLowLevel.MotorType.kBrushless);
//     }

//     @Override
//     public void execute(){
//         //Shoots note out, speed can be changed. Motor One should be positive and Motor Two should be negative
//         shooter.shooterMotorOne.set(in);
//         shooter.shooterMotorTwo.set(-in);
//     }


//     @Override
//     public void end(boolean interrupted){
//         shooterMotorOne.set(0);
//         shooterMotorTwo.set(0);
//     }
// }
