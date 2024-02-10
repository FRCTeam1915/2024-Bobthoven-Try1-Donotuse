package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class intakeArm extends CommandBase{
    // Intake armMotorOne;
    // Intake armMotorTwo;
    public static TalonFX armMotorOne;
    public static TalonFX armMotorTwo;
    double in;

    public intakeArm(double tin){
        in = tin;
        armMotorOne = new TalonFX(Constants.armMotorOne);
        armMotorTwo = new TalonFX(Constants.armMotorTwo);
        // addRequirements(armMotorOne);
    }

    @Override
    public void execute(){
        armMotorOne.set(-in);
        armMotorTwo.set(in);
    }

    @Override
    public void end(boolean interrupted){
        armMotorOne.set(0);
        armMotorTwo.set(0);
    }

    
    @Override
    public boolean isFinished(){
        return false;
    }
}
