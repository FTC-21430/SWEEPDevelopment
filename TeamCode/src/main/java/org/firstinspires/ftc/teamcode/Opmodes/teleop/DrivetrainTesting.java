package org.firstinspires.ftc.teamcode.Opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.TemplateBot;

/**
 * Driver-operated drivetrain test.
 * The left stick supplies horizontal and vertical inputs (x strafe, y forward); right-stick x rotates.
 * The current drivetrain mixer maps x to its forward argument and y to its strafe argument, as documented there.
 */
@TeleOp(name = "DrivetrainTesting", group = "Teleop-Testing")
public class DrivetrainTesting extends LinearOpMode {
    /** Runs the drivetrain using the primary gamepad sticks until the opmode stops. */
    @Override
    public void runOpMode(){
        TemplateBot robot = new TemplateBot(hardwareMap, telemetry);
        robot.initialize();
        robot.getDrivetrain().enableFieldCentricDriving(false);
        waitForStart();
        while(opModeIsActive()){
            robot.getDrivetrain().updateMovementVector(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);
            robot.teleopUpdate();
        }
    }
}
