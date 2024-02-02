package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {

    public static TalonSRX lowerMotor;
    public static TalonSRX upperMotor;
    public static TalonSRX shooterMotorOne;
    public static TalonSRX shooterMotorTwo;

    public static TalonSRX intake;
    public Intake() {
        //Defines the upper and lower motors for the the shooter and intake
        lowerMotor = new TalonSRX(Constants.pickUpID);
        upperMotor = new TalonSRX(Constants.upperMotorID);
        shooterMotorOne = new TalonSRX(Constants.shooterOneID);
        shooterMotorTwo = new TalonSRX(Constants.shooterTwoID);
        
    }

    @Override
    public void periodic() {

    }

    public void stop() {

    }
}
