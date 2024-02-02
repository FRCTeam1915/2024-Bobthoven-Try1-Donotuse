package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class shooter extends CommandBase{
    Intake upperMotor;
    boolean in;
    public shooter(Intake upperMotor, boolean tin){
        in = tin;
        addRequirements(upperMotor);
    }

    @Override
    public void execute(){
        //Shoots note out, speed can be changed
        Intake.upperMotor.set(ControlMode.PercentOutput, -1);
    }
}
