package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {

    public static TalonSRX lowerMotor;
    public static TalonSRX upperMotor;

    public static TalonSRX intake;
    public Intake() {
        //lowerMotor = new TalonSRX(Constants.pickUpID);
        upperMotor = new TalonSRX(Constants.shooterID);
        
    }

    @Override
    public void periodic() {

    }

    public void stop() {
        
    }
}
