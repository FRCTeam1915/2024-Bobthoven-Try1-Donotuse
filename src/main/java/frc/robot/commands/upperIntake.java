package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class upperIntake extends CommandBase{
    // Intake lowerMotor;
    Intake upperMotor;
    Intake lowerMotor;
    boolean in;

    public upperIntake(Intake upperMotor, Intake lowerMotor, boolean tin){
        in = tin;
        addRequirements(upperMotor);

    }


    @Override
    public void execute(){
        //Pick up speed for intake
        // Intake.lowerMotor.set(ControlMode.PercentOutput, .5);
        Intake.upperMotor.set(ControlMode.PercentOutput, .5);
        Intake.lowerMotor.set(ControlMode.PercentOutput, .5);
    }


    @Override
    public void end(boolean interrupted){
        //Intake.lowerMotor.set(ControlMode.PercentOutput,0);
        Intake.upperMotor.set(ControlMode.PercentOutput, 0);
        Intake.lowerMotor.set(ControlMode.PercentOutput, 0);
    }


    @Override
    public boolean isFinished(){
        return false;
    }
}
