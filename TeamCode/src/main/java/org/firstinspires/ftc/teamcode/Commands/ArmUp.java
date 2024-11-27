package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;


public class ArmUp extends CommandBase {
    private final Arm arm;
    public ArmUp(Arm arm) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.arm = arm;
        addRequirements(arm);
        Log.d("AUTONOMO","Arm up Inicio");
    }
    @Override
    public void initialize() {

        arm.setArm(1);
        arm.setElevator(1);
    }
    @Override
    public void end(boolean interrupted){
        Log.d("AUTONOMO","Arm up Fim");
    }
    @Override
    public boolean isFinished() {

        return (arm.isArmUp() && arm.isElevatorUp());
    }
}
