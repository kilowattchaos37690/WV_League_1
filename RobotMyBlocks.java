package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

/**
 * Kid-facing myBlocks for test_config.xml.
 *
 * Coaches tune PIDF / inches constants below. Kids only drag blocks like
 * "Drive Forward" — they never see PIDF.
 *
 * In Blocks: call {@link #initRobot()} once after waitForStart (or before).
 * In Java OpModes: call {@link #bindFromLinearOpMode} first, then initRobot().
 */
public class RobotMyBlocks extends BlocksOpModeCompanion {

  // --- Coach tuning (not exported) ---
  /**
   * When false, skip lift_up / left_servo / right_servo in {@link #initRobot} and
   * make lift/servo myBlocks no-ops. Use false for drive-only testing when those
   * devices are not in the active robot config. Set true when lift + servos are wired.
   */
  private static final boolean ENABLE_LIFT_AND_SERVOS = false;

  /** NeveRest 40 counts per motor revolution (typical). */
  private static final double COUNTS_PER_MOTOR_REV = 1120.0;
  /** Drive gearing: 1.0 if wheel on motor shaft; change if geared. */
  private static final double DRIVE_GEAR = 1.0;
  /** Wheel diameter inches — MEASURE and update. */
  private static final double WHEEL_DIAMETER_IN = 4.0;
  private static final double COUNTS_PER_INCH =
      (COUNTS_PER_MOTOR_REV * DRIVE_GEAR) / (Math.PI * WHEEL_DIAMETER_IN);
  /** Rough inches of wheel travel for a 90° spin in place — tune on carpet. */
  private static final double INCHES_PER_90_DEG_TURN = 12.0;

  /**
   * TeleOp stick feel (coach-tuned). Applied inside {@link #driveArcade}.
   * 1.0 = full stick → full power; 0.5 ≈ half as twitchy; cubic softens small stick moves.
   */
  private static final double TELEOP_SCALE = 0.55;
  /** 1 = linear; 3 = cubic (gentle near center, full at stick extremes). */
  private static final int TELEOP_EXPO = 3;

  // Velocity PIDF (RUN_USING_ENCODER / RUN_TO_POSITION). Tune on the robot.
  private static final double DRIVE_P = 10.0;
  private static final double DRIVE_I = 0.05;
  private static final double DRIVE_D = 0.0;
  private static final double DRIVE_F = 0.0;
  private static final double DRIVE_POS_P = 5.0;

  private static final double LIFT_P = 10.0;
  private static final double LIFT_I = 0.05;
  private static final double LIFT_D = 0.0;
  private static final double LIFT_F = 0.0;
  private static final double LIFT_POS_P = 5.0;

  private static final int POS_TOLERANCE = 20;
  private static final double DEFAULT_TIMEOUT_SEC = 5.0;
  /**
   * Soft-start for encoder Auto: seconds to ramp 0 → commanded power (0 = off).
   * Cuts the initial jerk on driveForward / turn / strafe.
   */
  private static final double POWER_RAMP_SEC = 0.20;

  private static DcMotorEx left_front;
  private static DcMotorEx left_back;
  private static DcMotorEx right_front;
  private static DcMotorEx right_back;
  private static DcMotorEx lift_up;
  private static Servo left_servo;
  private static Servo right_servo;
  private static boolean ready = false;

  /** Call from Java LinearOpModes so myBlocks see this OpMode's hardware. */
  public static void bindFromLinearOpMode(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode op) {
    opMode = op;
    linearOpMode = op;
    hardwareMap = op.hardwareMap;
    telemetry = op.telemetry;
    gamepad1 = op.gamepad1;
    gamepad2 = op.gamepad2;
  }

  @ExportToBlocks(
      heading = "Robot",
      comment = "Init once — maps test_config devices and sets PIDF",
      tooltip = "Call at the start of Auto or TeleOp")
  public static void initRobot() {
    left_front = hardwareMap.get(DcMotorEx.class, "left_front");
    left_back = hardwareMap.get(DcMotorEx.class, "left_back");
    right_front = hardwareMap.get(DcMotorEx.class, "right_front");
    right_back = hardwareMap.get(DcMotorEx.class, "right_back");

    left_front.setDirection(DcMotor.Direction.REVERSE);
    left_back.setDirection(DcMotor.Direction.REVERSE);
    right_front.setDirection(DcMotor.Direction.FORWARD);
    right_back.setDirection(DcMotor.Direction.FORWARD);

    DcMotorEx[] drive = {left_front, left_back, right_front, right_back};
    for (DcMotorEx m : drive) {
      m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      applyDrivePidf(m);
      m.setTargetPositionTolerance(POS_TOLERANCE);
    }

    // Hold drive at 0 so PIDF apply does not twitch wheels at init.
    setDrivePower(0);

    if (ENABLE_LIFT_AND_SERVOS) {
      lift_up = hardwareMap.get(DcMotorEx.class, "lift_up");
      left_servo = hardwareMap.get(Servo.class, "left_servo");
      right_servo = hardwareMap.get(Servo.class, "right_servo");

      lift_up.setDirection(DcMotor.Direction.FORWARD);
      lift_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      lift_up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      applyLiftPidf(lift_up);
      lift_up.setTargetPositionTolerance(POS_TOLERANCE);

      left_servo.setPosition(0.5);
      right_servo.setPosition(0.5);
    } else {
      lift_up = null;
      left_servo = null;
      right_servo = null;
      if (telemetry != null) {
        telemetry.addData(
            "RobotMyBlocks",
            "Lift/servos OFF (ENABLE_LIFT_AND_SERVOS=false) — drive only");
      }
    }

    ready = true;
  }

  /**
   * Custom PIDF needs Control/Expansion Hub firmware that supports PIDF commands
   * (historically ≥ 1.8.2). Older firmware throws UnsupportedOperationException —
   * catch it so TeleOp/Auto still run on default hub coefficients.
   */
  private static void applyDrivePidf(DcMotorEx m) {
    try {
      m.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F);
      m.setPositionPIDFCoefficients(DRIVE_POS_P);
    } catch (UnsupportedOperationException e) {
      if (telemetry != null) {
        telemetry.addData(
            "PIDF",
            "Hub rejected custom drive PIDF — update hub firmware (REV Hardware Client). Using defaults.");
      }
    }
  }

  private static void applyLiftPidf(DcMotorEx m) {
    try {
      m.setVelocityPIDFCoefficients(LIFT_P, LIFT_I, LIFT_D, LIFT_F);
      m.setPositionPIDFCoefficients(LIFT_POS_P);
    } catch (UnsupportedOperationException e) {
      if (telemetry != null) {
        telemetry.addData(
            "PIDF",
            "Hub rejected custom lift PIDF — update hub firmware. Using defaults.");
      }
    }
  }

  private static void ensureReady() {
    if (!ready) {
      initRobot();
    }
  }

  private static boolean liftServosReady() {
    return ENABLE_LIFT_AND_SERVOS && lift_up != null && left_servo != null && right_servo != null;
  }

  // -------------------------------------------------------------------------
  // Drive — kids blocks
  // -------------------------------------------------------------------------

  @ExportToBlocks(
      heading = "Drive",
      comment = "Drive forward (+) or back (−) using encoders + PIDF",
      parameterLabels = {"Inches", "Max power 0..1"})
  public static void driveForward(double inches, double power) {
    ensureReady();
    encoderDrive(inches, inches, inches, inches, power, DEFAULT_TIMEOUT_SEC);
  }

  @ExportToBlocks(
      heading = "Drive",
      comment = "Strafe right (+) or left (−) — mecanum",
      parameterLabels = {"Inches", "Max power 0..1"})
  public static void strafeRight(double inches, double power) {
    ensureReady();
    // LF+, LB−, RF−, RB+ for right strafe (sign follows common mecanum mix)
    encoderDrive(inches, -inches, -inches, inches, power, DEFAULT_TIMEOUT_SEC);
  }

  @ExportToBlocks(
      heading = "Drive",
      comment = "Spin right (+) or left (−) degrees (encoder estimate — tune)",
      parameterLabels = {"Degrees", "Max power 0..1"})
  public static void turnDegrees(double degrees, double power) {
    ensureReady();
    double inches = (degrees / 90.0) * INCHES_PER_90_DEG_TURN;
    // Left forward, right back for CW/right turn depending on directions
    encoderDrive(inches, inches, -inches, -inches, power, DEFAULT_TIMEOUT_SEC);
  }

  @ExportToBlocks(
      heading = "Drive",
      comment = "TeleOp mecanum: forward, strafe, turn in −1..1 (scaled by TELEOP_SCALE / EXPO)",
      parameterLabels = {"Forward", "Strafe", "Turn"})
  public static void driveArcade(double forward, double strafe, double turn) {
    ensureReady();
    setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    forward = shapeStick(forward);
    strafe = shapeStick(strafe);
    turn = shapeStick(turn);
    double fl = forward + strafe + turn;
    double bl = forward - strafe + turn;
    double fr = forward - strafe - turn;
    double br = forward + strafe - turn;
    double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
    left_front.setPower(fl / max);
    left_back.setPower(bl / max);
    right_front.setPower(fr / max);
    right_back.setPower(br / max);
  }

  /** Deadzone-ish expo + overall scale for less twitchy Driver Station sticks. */
  private static double shapeStick(double v) {
    double s = Math.max(-1.0, Math.min(1.0, v));
    if (TELEOP_EXPO == 3) {
      s = s * s * s;
    } else if (TELEOP_EXPO > 1) {
      s = Math.copySign(Math.pow(Math.abs(s), TELEOP_EXPO), s);
    }
    return s * TELEOP_SCALE;
  }

  @ExportToBlocks(
      heading = "Drive",
      comment = "Stop drive motors (call after an Auto sequence of driveForward / turn)")
  public static void stopDrive() {
    ensureReady();
    setDrivePower(0);
    // TeleOp / idle: open-loop. Encoder moves leave RUN_TO_POSITION between segments.
    setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  // -------------------------------------------------------------------------
  // Lift + servos
  // -------------------------------------------------------------------------

  @ExportToBlocks(
      heading = "Lift",
      comment = "Move lift by encoder inches (approx) with PIDF",
      parameterLabels = {"Inches", "Max power 0..1"})
  public static void liftMoveInches(double inches, double power) {
    ensureReady();
    if (!liftServosReady()) {
      return;
    }
    int target = lift_up.getCurrentPosition() + (int) Math.round(inches * COUNTS_PER_INCH);
    lift_up.setTargetPosition(target);
    lift_up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lift_up.setPower(Math.abs(power));
    ElapsedTime timer = new ElapsedTime();
    while (opModeIsActiveCompat() && lift_up.isBusy() && timer.seconds() < DEFAULT_TIMEOUT_SEC) {
      idleCompat();
    }
    lift_up.setPower(0);
    lift_up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  @ExportToBlocks(
      heading = "Lift",
      comment = "TeleOp lift power −1..1 (no PID hold)",
      parameterLabels = {"Power"})
  public static void liftPower(double power) {
    ensureReady();
    if (!liftServosReady()) {
      return;
    }
    lift_up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    lift_up.setPower(power);
  }

  @ExportToBlocks(heading = "Lift", comment = "Stop lift motor")
  public static void liftStop() {
    ensureReady();
    if (!liftServosReady()) {
      return;
    }
    lift_up.setPower(0);
  }

  @ExportToBlocks(
      heading = "Servo",
      comment = "left_servo position 0..1",
      parameterLabels = {"Position"})
  public static void leftServo(double position) {
    ensureReady();
    if (!liftServosReady()) {
      return;
    }
    left_servo.setPosition(clamp01(position));
  }

  @ExportToBlocks(
      heading = "Servo",
      comment = "right_servo position 0..1",
      parameterLabels = {"Position"})
  public static void rightServo(double position) {
    ensureReady();
    if (!liftServosReady()) {
      return;
    }
    right_servo.setPosition(clamp01(position));
  }

  @ExportToBlocks(heading = "Servo", comment = "Open-ish poses (0.8)")
  public static void servosOpen() {
    ensureReady();
    if (!liftServosReady()) {
      return;
    }
    left_servo.setPosition(0.8);
    right_servo.setPosition(0.8);
  }

  @ExportToBlocks(heading = "Servo", comment = "Closed-ish poses (0.2)")
  public static void servosClose() {
    ensureReady();
    if (!liftServosReady()) {
      return;
    }
    left_servo.setPosition(0.2);
    right_servo.setPosition(0.2);
  }

  // -------------------------------------------------------------------------
  // Internals (hidden from Blocks)
  // -------------------------------------------------------------------------

  private static void encoderDrive(
      double lfIn, double lbIn, double rfIn, double rbIn, double power, double timeoutSec) {
    int newLf = left_front.getCurrentPosition() + (int) Math.round(lfIn * COUNTS_PER_INCH);
    int newLb = left_back.getCurrentPosition() + (int) Math.round(lbIn * COUNTS_PER_INCH);
    int newRf = right_front.getCurrentPosition() + (int) Math.round(rfIn * COUNTS_PER_INCH);
    int newRb = right_back.getCurrentPosition() + (int) Math.round(rbIn * COUNTS_PER_INCH);

    left_front.setTargetPosition(newLf);
    left_back.setTargetPosition(newLb);
    right_front.setTargetPosition(newRf);
    right_back.setTargetPosition(newRb);

    setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    double p = Math.abs(power);

    ElapsedTime timer = new ElapsedTime();
    while (opModeIsActiveCompat()
        && timer.seconds() < timeoutSec
        && (left_front.isBusy() || left_back.isBusy() || right_front.isBusy() || right_back.isBusy())) {
      double pNow = p;
      if (POWER_RAMP_SEC > 0 && timer.seconds() < POWER_RAMP_SEC) {
        pNow = p * (timer.seconds() / POWER_RAMP_SEC);
      }
      setDrivePower(pNow);
      idleCompat();
    }
    // Soft end for chaining: zero power but stay in RUN_TO_POSITION so the next
    // driveForward/turn can retarget without an open-loop mode reset (that pause).
    // Call stopDrive() once when the whole Auto sequence is finished.
    setDrivePower(0);
  }

  private static void setDrivePower(double p) {
    left_front.setPower(p);
    left_back.setPower(p);
    right_front.setPower(p);
    right_back.setPower(p);
  }

  private static void setDriveRunMode(DcMotor.RunMode mode) {
    left_front.setMode(mode);
    left_back.setMode(mode);
    right_front.setMode(mode);
    right_back.setMode(mode);
  }

  private static boolean opModeIsActiveCompat() {
    return linearOpMode == null || linearOpMode.opModeIsActive();
  }

  private static void idleCompat() {
    if (linearOpMode != null) {
      linearOpMode.idle();
    }
  }

  private static double clamp01(double v) {
    if (v < 0) return 0;
    if (v > 1) return 1;
    return v;
  }
}
