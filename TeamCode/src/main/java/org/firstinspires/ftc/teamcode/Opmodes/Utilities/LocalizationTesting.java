package org.firstinspires.ftc.teamcode.Opmodes.Utilities;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.TemplateBot;

/**
 * Displays the robot's GoBilda localization readings for a driver-operated test.
 * The driver can move the robot and observe X/Y position in inches and heading in degrees on telemetry.
 */
@TeleOp(name = "LocalizationTesting", group = "Utility-Testing")
public class LocalizationTesting extends LinearOpMode {
    /** Updates localization and reports its position and heading until the opmode stops. */
    @Override
    public void runOpMode() throws InterruptedException {
        TemplateBot robot = new TemplateBot(hardwareMap, telemetry);
        robot.initialize();
        waitForStart();
        robot.localization.overrideLocalization(0,0,0);
        while (opModeIsActive()) {
            robot.localization.update();
            LocalizationPacket currentPacket = robot.localization.getLocalizationPacket();
            telemetry.addData("X", currentPacket.getX());
            telemetry.addData("Y", currentPacket.getY());
            telemetry.addData("Heading", currentPacket.getYaw());
            telemetry.addLine("--------------------- \n");

        }
    }
}
