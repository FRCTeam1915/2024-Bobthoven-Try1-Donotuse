package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class pickUp extends CommandBase{
    Intake lowerMotor;
    boolean in;

    public pickUp(Intake lowerMotor, boolean tin){
        in = tin;
        addRequirements(lowerMotor);
    }


    @Override
    public void execute(){
        Intake.intake.set(ControlMode.PercentOutput, .5);
    }


    @Override
    public void end(boolean interrupted){
        Intake.intake.set(ControlMode.PercentOutput,0);
    }


    @Override
    public boolean isFinished(){
        return false;
    }
}
