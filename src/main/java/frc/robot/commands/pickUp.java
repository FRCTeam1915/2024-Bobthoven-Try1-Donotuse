package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class pickUp extends CommandBase{
    // Intake lowerMotor;
    public static TalonSRX lowerMotor;
    public static TalonSRX upperMotor;
    // Intake upperMotor;
    boolean in;

    public pickUp(boolean tin){
        in = tin;
        lowerMotor = new TalonSRX(Constants.pickUpID);
        upperMotor = new TalonSRX(Constants.upperMotorID);
        // addRequirements(lowerMotor);

    }


    @Override
    public void execute(){
        //Pick up speed for intake
        lowerMotor.set(ControlMode.PercentOutput, -.75);
        // Intake.upperMotor.set(ControlMode.PercentOutput, .5);
    }


    @Override
    public void end(boolean interrupted){
        lowerMotor.set(ControlMode.PercentOutput,0);
    }


    @Override
    public boolean isFinished(){
        return false;
    }
}
