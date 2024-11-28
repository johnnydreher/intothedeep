package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;


public class ElevatorUp extends CommandBase {
    private final Arm arm;
    public ElevatorUp(Arm arm) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.arm = arm;
        addRequirements(arm);
        Log.d("AUTONOMO","Arm Down Inicio");
    }
    @Override
    public void initialize() {

        arm.setElevator(1);
    }

    @Override
    public void end(boolean interrupted){
        Log.d("AUTONOMO","Arm Down Fim");
    }
    @Override
    public boolean isFinished() {

        return arm.isElevatorUp();
    }
}
