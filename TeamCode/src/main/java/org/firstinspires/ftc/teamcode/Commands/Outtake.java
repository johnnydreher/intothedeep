package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class Outtake extends CommandBase {
    private final Intake intake;
    private final ElapsedTime timer;
    private boolean isFinished = false;
    public Outtake(Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intake = intake;
        addRequirements(intake);
        timer = new ElapsedTime();
    }
    @Override
    public void initialize() {
        intake.up();
        timer.reset();

    }
    @Override
    public void execute() {
        if(timer.milliseconds() > 500 && !isFinished){
            intake.push();
            isFinished = true;
        }
    }
    @Override
    public void end(boolean interrupted) {
        intake.powerOff();
    }
    @Override
    public boolean isFinished() {
        return timer.milliseconds()>1500;

    }
}
