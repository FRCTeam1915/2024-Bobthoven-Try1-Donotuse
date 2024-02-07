package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class intakeArm extends CommandBase{
    Intake armMotorOne;
    Intake armMotorTwo;
    double in;

    public intakeArm(Intake armMotorOne, Intake armMotorTwo, double tin){
        in = tin;
        addRequirements(armMotorOne);
    }

    @Override
    public void execute(){
        Intake.shooterMotorOne.set(ControlMode.PercentOutput, in);
        Intake.shooterMotorOne.set(ControlMode.PercentOutput, in);
    }


    @Override
    public boolean isFinished(){
        return false;
    }
}
