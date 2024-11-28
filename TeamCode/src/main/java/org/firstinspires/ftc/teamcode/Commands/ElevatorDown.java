package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;


public class ElevatorDown extends CommandBase {
    private final Arm arm;
    public ElevatorDown(Arm arm) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.arm = arm;
        addRequirements(arm);
        Log.d("AUTONOMO","Arm Down Inicio");
    }
    @Override
    public void initialize() {

        arm.setElevatorZero();
    }

    @Override
    public void end(boolean interrupted){

        Log.d("AUTONOMO","Arm Down Fim");
    }
    @Override
    public boolean isFinished() {

        return arm.getElevatorEncoder()<4000;
    }
}
