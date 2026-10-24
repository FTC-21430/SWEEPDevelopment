package org.firstinspires.ftc.teamcode.Opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.broombots.sweep.Builder.SequenceBuilder;
import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExampleCode.SWEEPRoute;
import org.firstinspires.ftc.teamcode.Robot.DebuggingRobotMovementParameters;
import org.firstinspires.ftc.teamcode.Robot.TemplateBot;

@Config
@Autonomous
public class StraightLineTest extends SWEEPRoute {
    TemplateBot r;
    @Override
    public void defineRoute(){
        r = new TemplateBot(hardwareMap, telemetry); // create the new robot object, passing FIRSTSDK bits to the constructor.
        super.robot = r; // Assigning the your robot the the superclasses robot, which it will then use to run the route on.
        super.sequence = new SequenceBuilder(new DebuggingRobotMovementParameters()) // create the path that will be executed.
                .start(0, 0, 0) // All paths must have a start point as the first
                .splineToAngle(0,40,0,1)
                .waitAt(2)
                .splineToAngle(0,0,0,1)
                .addBreak()
                .splineToAngle(0,40,180,1)
                .waitAt(2)
                .splineToAngle(0,0,0,1)
                .splineToAngle(0,40,0,1)
                .waitAt(2)
                .splineToAngle(0,0,0,1)
                .addBreak()
                .splineToAngle(0,40,180,1)
                .waitAt(2)
                .splineToAngle(0,0,0,1)
                .splineToAngle(0,40,0,1)
                .waitAt(2)
                .splineToAngle(0,0,0,1)
                .addBreak()
                .splineToAngle(0,40,180,1)
                .waitAt(2)
                .splineToAngle(0,0,0,1)
                .splineToAngle(0,40,0,1)
                .waitAt(2)
                .splineToAngle(0,0,0,1)
                .addBreak()
                .end(0, 0, 0) // robot will move to the end point with the specified coordinates and heading
                .build(); // finalizes the path and returns a Path object
   }
   @Override
    public void runOpMode(){
       defineRoute();
       if (sequence == null) throw new RuntimeException("Path is not defined. Override defineRoute() method and create a path.");
       if (robot == null) throw new RuntimeException("Robot is not defined. Override defineRoute() method and create a robot.");
       robot.initialize();
       waitForStart();
       robot.sequenceInterpreter.startPath(sequence);
       while (opModeIsActive()) {
           robot.localization.update();
           LocalizationPacket localizationPacket = robot.localization.getLocalizationPacket();
           Pos2D movementVector = robot.sequenceInterpreter.update(localizationPacket);
           PathPoint targetPathPoint = robot.sequenceInterpreter.getPathPoint();
           r.getDrivetrain().updateMovementVector(movementVector.x,movementVector.y,movementVector.angle);
           r.getDrivetrain().update();
           telemetry.addData("driveX", movementVector.x);
           telemetry.addData("driveY", movementVector.y);
           telemetry.addData("driveAngle", movementVector.angle);
           telemetry.addLine("------------- \n");
           telemetry.addData("X", localizationPacket.getX());
           telemetry.addData("Y", localizationPacket.getY());
           telemetry.addData("Heading", localizationPacket.getYaw());

           telemetry.addLine("------------- \n");
           telemetry.addData("targetX", targetPathPoint.position.x);
           telemetry.addData("targetY", targetPathPoint.position.y);
           telemetry.addData("targetAngle", targetPathPoint.position.angle);
           telemetry.update();

       }
   }
}
