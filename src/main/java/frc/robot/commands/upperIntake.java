package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class upperIntake extends CommandBase{
    // Intake lowerMotor;
    public static TalonSRX lowerMotor;
    public static TalonSRX upperMotor;
    // Intake upperMotor;
    // Intake lowerMotor;
    boolean in;

    public upperIntake(){
        lowerMotor = new TalonSRX(Constants.pickUpID);
        upperMotor = new TalonSRX(Constants.upperMotorID);

    }


    @Override
    public void execute(){
        //Pick up speed for intake
        // Intake.lowerMotor.set(ControlMode.PercentOutput, .5);
        upperMotor.set(ControlMode.PercentOutput, -.75);
        lowerMotor.set(ControlMode.PercentOutput, .75);
    }


    @Override
    public void end(boolean interrupted){
        //Intake.lowerMotor.set(ControlMode.PercentOutput,0);
        upperMotor.set(ControlMode.PercentOutput, 0);
        lowerMotor.set(ControlMode.PercentOutput, 0);
    }


    @Override
    public boolean isFinished(){
        return false;
    }
}
