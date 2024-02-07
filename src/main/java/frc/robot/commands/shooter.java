package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class shooter extends CommandBase{
    Intake shooterMotorOne;
    Intake shooterMotorTwo;
    boolean in;
    public shooter(Intake shooterMotorOne, Intake shooterMotorTwo, boolean tin) {
        in = tin;
       // addRequirements(upperMotor);
    }

    @Override
    public void execute(){
        //Shoots note out, speed can be changed
        System.out.println("Shooter ----------");
        Intake.shooterMotorOne.set(ControlMode.PercentOutput, 1);
        Intake.shooterMotorTwo.set(ControlMode.PercentOutput, -1);
    }


    @Override
    public void end(boolean interrupted){
        Intake.shooterMotorOne.set(ControlMode.PercentOutput,0);
        Intake.shooterMotorTwo.set(ControlMode.PercentOutput,0);
    }
}
