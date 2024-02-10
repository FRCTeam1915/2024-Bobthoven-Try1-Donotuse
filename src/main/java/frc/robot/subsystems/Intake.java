package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {

    // public static TalonSRX lowerMotor;
    // public static TalonSRX upperMotor;
    //public static TalonFX shooterMotorOne;
    //public static TalonFX shooterMotorTwo;
    
    // public static TalonFX armMotorOne;
    // public static TalonFX armMotorTwo;

    public static TalonSRX intake;
    public Intake() {
        //Defines the upper and lower motors for the the shooter and intake
        // lowerMotor = new TalonSRX(Constants.pickUpID);
        // upperMotor = new TalonSRX(Constants.upperMotorID);
        //shooterMotorOne = new TalonFX(Constants.shooterOneID);
        //shooterMotorTwo = new TalonFX(Constants.shooterTwoID);
        
        
        // armMotorOne = new TalonFX(Constants.armMotorOne);
        // armMotorTwo = new TalonFX(Constants.armMotorTwo);
        
    }

    @Override
    public void periodic() {

    }

    public void stop() {

    }
}
