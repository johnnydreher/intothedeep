package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class Grab extends CommandBase {
    private final Intake intake;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isFinished = false;
    public Grab(Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        intake.down();
        timer.reset();


    }
    @Override
    public void execute() {
        if(timer.milliseconds() > 500){
            intake.pull();
            //isFinished = true;
        }
    }
    @Override
    public void end(boolean interrupted) {
        intake.powerOff();
    }
    @Override
    public boolean isFinished() {
        // TODO: 19/11/2024 Adicionar controle por sensor
        return timer.milliseconds()>1000;
    }
}
