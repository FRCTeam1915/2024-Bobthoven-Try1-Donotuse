package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.Robot;

public class shooter extends CommandBase{
    public static CANSparkFlex shooterMotorOne = new CANSparkFlex(Constants.shooterMotorOne, CANSparkLowLevel.MotorType.kBrushless);
    public static CANSparkFlex shooterMotorTwo = new CANSparkFlex(Constants.shooterMotorTwo, CANSparkLowLevel.MotorType.kBrushless);
    
    // Intake shooterMotorOne;
    // Intake shooterMotorTwo;
    double in;
    public shooter(double tin, double rawValue) {
        in = tin;
        // double rawValue = ultrasonic.getValue();
        //shooterMotorOne = new CANSparkFlex(Constants.shooterMotorOne, CANSparkLowLevel.MotorType.kBrushless);
        // shooterMotorTwo = new CANSparkFlex(Constants.shooterMotorTwo, CANSparkLowLevel.MotorType.kBrushless);
    }

    @Override
    public void execute(){
        
        //Shoots note out, speed can be changed. Motor One should be positive and Motor Two should be negative
        shooterMotorOne.set(in);
        shooterMotorTwo.set(-in);
        // SmartDashboard.setDefaultNumber("Ultrasonic Sensor", rawValue);
        
        
    }


    @Override
    public void end(boolean interrupted){
        shooterMotorOne.set(0);
        shooterMotorTwo.set(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
