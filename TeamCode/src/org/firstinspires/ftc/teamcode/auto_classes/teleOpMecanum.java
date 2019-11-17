package org.firstinspires.ftc.teamcode.auto_classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.general_classes.Position2DAngle;
import org.firstinspires.ftc.teamcode.team_classes.Robot;

@TeleOp(name="Mecanum", group="9108OpMode") //fix this
public class teleOpMecanum extends OpMode {

    //private ElapsedTime runtime = new ElapsedTime();
    private Robot Robot = new Robot();

    //Initialized by: Initialization Button (i think)
    public void init() {
        //telemetry.addData("Status", "Initializing");
        //telemetry.update();
        Robot.initialize(hardwareMap,telemetry);
        //telemetry.addData("Status","Complete");
        //telemetry.update();
    }

    //Initialized by: Start / runs once
    @Override
    public void start() {
        //runtime.reset();
    }


    //Initialized by: After Start, Before Stop / loops
    @Override
    public void loop() {
        double drivey = gamepad1.left_stick_y;
        double drivex = gamepad1.left_stick_x;
        double turn  =  -gamepad1.right_stick_x;
        if (Math.abs(drivey) < .05) {
            drivey = 0;
        }
        if (Math.abs(drivex) < .05) {
            drivex = 0;
        }
        if (Math.abs(turn) < .05) {
            turn = 0;
        }
        Position2DAngle relativeValues;
        relativeValues = Robot.DCG.relativeValues(new Position2DAngle(drivex,drivey,turn), Robot.IMU);
        Robot.DCG.driveToPositionAngle(relativeValues, true);
        //Robot.DCG.driveToPositionAngle(new Position2DAngle(drivex,drivey,turn), true);
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.update();
        //Robot.DCG.setPower(new double[]{0,100,0,100});
        //Robot.DCG.driveToPositionAngle(new Position2DAngle(100,100,0),true);
    }

    //Initialized by: Stop / runs once
    @Override
    public void stop() {
        Robot.DCG.setPower(new double[]{0,0,0,0});
    }
}
