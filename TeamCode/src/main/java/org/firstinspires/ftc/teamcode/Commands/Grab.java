package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class Grab extends CommandBase {
    private final Intake intake;
    public Grab(Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        intake.pull();
    }
    @Override
    public void execute() {
    }
    @Override
    public void end(boolean interrupted) {
        intake.powerOff();
    }
    @Override
    public boolean isFinished() {
        return intake.elementPresent();
    }
}
