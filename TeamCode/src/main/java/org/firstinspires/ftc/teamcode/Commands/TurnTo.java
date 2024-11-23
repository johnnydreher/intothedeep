package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class TurnTo extends CommandBase {
    Drivetrain drive;
    PIDControl pid;
    double angle;
    public TurnTo(double angle, Drivetrain drive){
        this.angle = angle;
        this.drive = drive;
        addRequirements(drive);
        pid = new PIDControl(angle, PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        drive.resetEncoders();
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        double out = pid.get(drive.getHeading(AngleUnit.DEGREES));
        drive.driveFieldCentric(0,0,-out);
    }
    @Override
    public void end(boolean interrupted) {
        drive.power(0);
    }
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}

