package org.firstinspires.ftc.teamcode.Commands;

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
        if(timer.milliseconds() > 500){
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
        // TODO: 19/11/2024 Adiconar controle por sensor
        return timer.milliseconds()>1000;
    }
}
