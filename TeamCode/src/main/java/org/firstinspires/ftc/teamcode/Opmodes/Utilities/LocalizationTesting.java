package org.firstinspires.ftc.teamcode.Opmodes.Utilities;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.TemplateBot;

@TeleOp
public class LocalizationTesting extends LinearOpMode {
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
