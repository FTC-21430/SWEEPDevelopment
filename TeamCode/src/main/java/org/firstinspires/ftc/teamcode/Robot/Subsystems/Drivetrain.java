package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.broombots.sweep.Classes.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drivetrain subsystem for a mecanum drive robot.
 * This class handles the control of the robot's drivetrain, including field-centric driving and telemetry.
 */
public class Drivetrain implements Subsystem {
    /** Front-left drive motor. */
    private final DcMotor flMotor;
    /** Front-right drive motor. */
    private final DcMotor frMotor;
    /** Back-left drive motor. */
    private final DcMotor blMotor;
    /** Back-right drive motor. */
    private final DcMotor brMotor;
    /** Drive motors ordered front-left, front-right, back-left, back-right. */
    private final DcMotor[] motors;
    /** Whether movement inputs are rotated using the current robot heading. */
    private boolean fieldCentricDrivingEnabled = false;
    /** Stored horizontal input power, unitless in the range -1 to 1. */
    private double inputX = 0;
    /** Stored vertical input power, unitless in the range -1 to 1. */
    private double inputY = 0;
    /** Stored rotation input power, unitless in the range -1 to 1. */
    private double inputYaw = 0;
    /** Current robot heading used for field-centric calculations, in degrees. */
    private double robotYaw = 0;
    /** Driver-station telemetry destination. */
    private final Telemetry telemetry;
    /** Whether drivetrain telemetry is posted during updates. */
    private boolean telemetryEnabled = false;
    /** Whether motor output is disabled. */
    private boolean disabled = false;

    /**
     * Constructor for the Drivetrain subsystem.
     * @param hardwareMap hardware map used to retrieve the four drive motors
     * @param telemetry telemetry destination for driver-station output
     * @param telemetryEnabled whether to post drivetrain telemetry during updates
     */
    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, boolean telemetryEnabled){
        flMotor = hardwareMap.get(DcMotor.class, "fl");
        frMotor = hardwareMap.get(DcMotor.class, "fr");
        blMotor = hardwareMap.get(DcMotor.class, "bl");
        brMotor = hardwareMap.get(DcMotor.class, "br");

        motors = new DcMotor[]{flMotor,frMotor,blMotor,brMotor};
        for (DcMotor motor: motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        this.telemetry = telemetry;
        this.telemetryEnabled = telemetryEnabled;
    }

    /**
     * Applies the stored movement command, optionally transforming a copy for field-centric driving, and posts telemetry.
     */
    @Override
    public void update() {
        // Apply the stored drive power every iteration
        // Rotate a copy so the stored input is not rotated again on the next loop
        double driveX = inputX;
        double driveY = inputY;
        if (fieldCentricDrivingEnabled){
            double[] fieldVector = calculateFieldCentric(robotYaw);
            driveX = fieldVector[0];
            driveY = fieldVector[1];
        }
        setDrivePower(driveX,driveY,inputYaw);
        if (telemetryEnabled){
            postDrivetrainTelemetry();
        }
    }
    /**
     * Stores a robot-relative movement vector and disables field-centric transformation.
     * The input convention is x for strafe and y for forward, but the existing mixer call passes x
     * as its forward argument and y as its strafe argument; this surprising ordering is preserved.
     * @param x horizontal movement input (intended strafe right when positive), unitless, -1 to 1
     * @param y vertical movement input (intended forward when positive), unitless, -1 to 1
     * @param yaw rotation input, unitless, -1 to 1
     */
    public void updateMovementVector(double x, double y, double yaw){
        fieldCentricDrivingEnabled = false;
        this.inputX = x;
        this.inputY = y;
        this.inputYaw = yaw;
    }
    /**
     * Stores a movement vector, enables field-centric transformation, and uses robotYaw in degrees to rotate the input.
     * The input convention is x for strafe and y for forward, but the existing mixer call passes x
     * as its forward argument and y as its strafe argument; this ordering is preserved.
     * @param x horizontal movement input (intended strafe right when positive), unitless, -1 to 1
     * @param y vertical movement input (intended forward when positive), unitless, -1 to 1
     * @param yaw rotation input, unitless, -1 to 1
     * @param robotYaw current robot heading used to rotate the input, in degrees
     */
    public void updateMovementVector(double x, double y, double yaw, double robotYaw){
        updateMovementVector(x,y,yaw); // this clears the field-centric flag, so set it after
        fieldCentricDrivingEnabled = true;
        this.robotYaw = robotYaw;
    }
    /**
     * Enable or disable field-centric driving.
     * @param enabled true to enable field-centric driving, false to disable
     */
    public void enableFieldCentricDriving(boolean enabled){
        this.fieldCentricDrivingEnabled = enabled;
    }

    /**
     * Set the disabled state of the drivetrain subsystem. Required function for all subsystems.
     * @param disable true to disable the subsystem, false to enable
     */
    @Override
    public void setDisabled(boolean disable){
        if (disable){
            for (DcMotor motor: motors){
                motor.setPower(0);
            }
        }
        this.disabled = disable;
    }
    /**
     * Calculate the field-centric movement vector based on the robot's current yaw angle.
     * @param angle current robot yaw angle in degrees
     * @return transformed x and y input powers, unitless
     */
    private double[] calculateFieldCentric(double angle){
        angle = AngleUnit.DEGREES.toRadians(angle); // convert to radians
        double fieldY = inputY * Math.cos(angle) + inputX * Math.sin(angle);
        double fieldX = -inputY * Math.sin(angle) + inputX * Math.cos(angle);
        return new double[]{fieldX,fieldY};
    }
    /**
     * Set the drive power for the drivetrain.
     * @param forward forward/backward movement power, unitless
     * @param strafe left/right movement power, unitless
     * @param rotate rotational movement power, unitless
     */
    private void setDrivePower(double forward, double strafe, double rotate) {
        if (disabled) return;
        // Mecanum drive: distribute robot forces to individual motors
        double[] powers = {
                forward + strafe + rotate, //FL
                forward - strafe + rotate, //FR
                forward - strafe - rotate, //BL
                forward + strafe - rotate  //BR
        };
        // Calculate the largest power magnitude out of all possible solutions, reverse powers count too
        double maxPower = 0;
        for (double power : powers){
            maxPower = Math.max(maxPower, Math.abs(power));
        }
        maxPower = maxPower > 1? maxPower : 1;
        // Apply motor power normalized to the greatest possible value
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i] / maxPower);
        }
    }
    /**
     * Post telemetry data for the drivetrain subsystem.
     */
    private void postDrivetrainTelemetry(){
        telemetry.addLine("---Drivetrain---");
        telemetry.addData("X Power", inputX);
        telemetry.addData("Y Power", inputY);
        telemetry.addData("Yaw Power", inputYaw);
        telemetry.addData("FCD Enabled", fieldCentricDrivingEnabled);
        telemetry.addData("FCD Angle", robotYaw);
    }
}
