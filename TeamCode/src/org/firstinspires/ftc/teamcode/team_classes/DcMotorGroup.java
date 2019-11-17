package org.firstinspires.ftc.teamcode.team_classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.general_classes.Position2D;
import org.firstinspires.ftc.teamcode.general_classes.Position2DAngle;

public class DcMotorGroup {
    //Properties
    public DcMotor[] DcMotors;
    public int DcMotorCount;

    //Constant Properties
    private static final double     COUNTS_PER_MOTOR_REV    = 0 ;       // 5202 Series Yellow Jacket Planetary Gear Motor
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_MM       = 100;      // goBilda metric wheels are 100mm
    private static final double     WHEEL_DIAMETER_INCHES   = WHEEL_DIAMETER_MM/25.4;
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //Constructor
    public DcMotorGroup(DcMotor[] DcMotorArray) {
        DcMotors = DcMotorArray;
        DcMotorCount = DcMotorArray.length;
    }

    //setPower to all of the motors
    public void setPower(double[] power) {
        for(int i=0;i<DcMotorCount;i++){
            this.DcMotors[i].setPower(power[i]);
        }
    }

    //METHOD 1: self-explanatory
    public void initialize(HardwareMap Hmap, Telemetry Tm) {
        Tm.addData("DcMotorGroup","Initializing");
        Tm.update();
        this.DcMotors[0] = Hmap.get(DcMotor.class, "front_right_motor");
        this.DcMotors[1] = Hmap.get(DcMotor.class, "front_left_motor");
        this.DcMotors[2] = Hmap.get(DcMotor.class, "back_left_motor");
        this.DcMotors[3] = Hmap.get(DcMotor.class, "back_right_motor");
        this.DcMotors[0].setDirection(DcMotor.Direction.REVERSE);
        this.DcMotors[1].setDirection(DcMotor.Direction.FORWARD);
        this.DcMotors[2].setDirection(DcMotor.Direction.FORWARD);
        this.DcMotors[3].setDirection(DcMotor.Direction.REVERSE);
        Tm.addData("Encoders","Resetting");
        Tm.update();
        this.DcMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.DcMotors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.DcMotors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.DcMotors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.DcMotors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.DcMotors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.DcMotors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.DcMotors[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Tm.addData("DcMotorGroup Initialization","Complete");
        Tm.update();
    }

    //X = a, Y = b
    //The distances are in inches
    private final double Xdistance = 1;
    private final double Ydistance = 1;
    private double XYcombinedD = Xdistance + Ydistance;

    public void driveToPositionAngle(Position2DAngle PositionAngle, boolean teleOp) {
        double relativeY = PositionAngle.Y;
        double relativeX = PositionAngle.X;
        double degreesAngle = PositionAngle.Angle;

        //NOTE: This uses displacement instead of velocity, since in practice the ratio of velocity_X to velocity_Y, will be equal to ratio of displacement_X to displacement_Y.
        double V1 = relativeY + relativeX - degreesAngle /* *(XYcombinedD)*/;
        double V2 = relativeY - relativeX + degreesAngle /* *(XYcombinedD)*/;
        double V3 = relativeY + relativeX + degreesAngle /* *(XYcombinedD)*/;
        double V4 = relativeY - relativeX - degreesAngle /* *(XYcombinedD)*/;

        double largest = Math.max(Math.max(V1,V2),Math.max(V3,V4));
        double smallest = Math.min(Math.min(V1,V2),Math.min(V3,V4));
        double divisor = Math.max(Math.abs(largest), Math.abs(smallest));

        if (divisor != 0) {
            V1 = 1*(V1/divisor);
            V2 = 1*(V2/divisor);
            V3 = 1*(V3/divisor);
            V4 = 1*(V4/divisor);
        }
        //double EncoderMax1 = inchToEncoder(0);

        this.setPower(new double[]{V1,V2,V3,V4});
        /*
        if (teleOp == false) {
            //WHILE ENCODER LOOP HERE
        } else if (teleOp == true) {

        }
         */
    }

    //IF undeclared teleop, assumes auto drive method
    public void driveToPositionAngle(Position2DAngle PositionAngle) {
        driveToPositionAngle(PositionAngle,false);
    }

    //METHOD 1.1: basically different input of method driveToPosition, named differently for distinguishing.
    public void strafeToAngle(Position2D Position, boolean teleOp) {
        double xlength = Position.X_POS;
        double ylength = Position.Y_POS;
        driveToPositionAngle(new Position2DAngle(xlength,ylength,0), teleOp);
    }

    //METHOD 1.2: ditto
    public void turnToAngle(double angle, boolean teleOp) {
        driveToPositionAngle(new Position2DAngle(0,0,angle), teleOp);
    }

    //DEPRECATED METHOD KEPT FOR EASE OF CODE
    public void driveToPosition(double X, double Y, double Angle) {
        driveToPositionAngle(new Position2DAngle(X,Y,Angle),false);
    }

    //FUNCTION 1: handles power scaling as robot approaches near target
    public double funcEncoderPercentagePower(double currentPosition, double encoderFinal, double maxPower) {
        int power = (int)(-100*(currentPosition/encoderFinal)+100);
        if (currentPosition>=encoderFinal){ power=0; }
        if (encoderFinal==0) { power = 0; }
        return power;
    }

    //FUNCTION 2: lots of trig going on, so have fun trying to figure it out
    public Position2DAngle relativeValues(Position2DAngle PosAngle, BNOIMU IMU) {
        double pass = PosAngle.Angle;
        double THETA_triangle;
        if (PosAngle.X==0 && PosAngle.Y!=0) {
            if (PosAngle.Y > 0) {
                THETA_triangle = 90;
            } else {
                THETA_triangle = -90;
            }
        } else if (PosAngle.X!=0 && PosAngle.Y==0) {
            if (PosAngle.X > 0) {
                THETA_triangle = 0;
            } else {
                THETA_triangle = 180;
            }
        } else if (PosAngle.X==0 && PosAngle.Y==0) {
            THETA_triangle = 0;
        } else {
            THETA_triangle = Math.toDegrees(Math.atan2(PosAngle.Y,PosAngle.X));
        }
        double THETA_relative = (THETA_triangle - IMU.getAngle());
        double L_hypotnuse = PosAngle.getMagnitude();
        double X_New = L_hypotnuse * Math.cos(Math.toRadians(THETA_relative));
        double Y_New = L_hypotnuse * Math.sin(Math.toRadians(THETA_relative));
        return new Position2DAngle(X_New,Y_New,pass);
    }

    //FUNCTION 3:
    public double inchToEncoder(double inches){
        return inches*COUNTS_PER_INCH;
    }

    //FUNCTION 4:
    public double EncoderRatioAngle(double encoder, double Angle){
        return 0;
    }

}
