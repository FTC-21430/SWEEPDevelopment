package org.firstinspires.ftc.teamcode.Opmodes.autonomous;

import com.broombots.sweep.Builder.SequenceBuilder;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExampleCode.SWEEPRoute;
import org.firstinspires.ftc.teamcode.Robot.TemplateBot;

@Autonomous
public class StraightLineTest extends SWEEPRoute {
    @Override
    public void defineRoute(){
        TemplateBot r = new TemplateBot(hardwareMap, telemetry); // create the new robot object, passing FIRSTSDK bits to the constructor.
        super.robot = r; // Assigning the your robot the the superclasses robot, which it will then use to run the route on.
        super.sequence = new SequenceBuilder(new DefaultRobotMovementParameters()) // create the path that will be executed.
                .start(0, 0, 0) // All paths must have a start point as the first
                .splineToAngle(0,10,0,1)
                .waitAt(50)
                .end(0, 0, 0) // robot will move to the end point with the specified coordinates and heading
                .build(); // finalizes the path and returns a Path object
   }
}
