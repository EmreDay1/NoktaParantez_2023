package frc.robot;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI;

import java.lang.Math;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Robot extends TimedRobot {

  private Timer myTimer;

  // KENDIME NOT: MOTORA VERILEN 1 VE -1 ARALIĞINDAKI DEĞERLER 12VOLTAJA ORAN

  // Tam emin değilim ancak Radyo için kullanıldığını düşünüyoruz. CHECK ET
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // REV Encoder (Duty Cycle encoder) returns absolute value
  private DutyCycleEncoder encoderJoint1;
  // NEO Encoder is relative encoder, resets everytime.
  private RelativeEncoder encoderJoint2;
  // Gearbox'ı oluşturan motorlar initialize
  private PWMVictorSPX motorJoint1a;
  private PWMVictorSPX motorJoint1b;
  // büyük baba control gear box
  private MotorControllerGroup motorJoint1;
  // SPARK initialize
  private CANSparkMax motorJoint2;
  private CANSparkMax intake;
  private XboxController xboxController;

  //initialize the switches
  private DigitalInput StopSwitch;
  //private DigitalInput limSwitch;
  private DigitalInput Joint2LowerSwitch;
  private boolean intakeOpen;
  private boolean quarter;



  //Chassis motorları 0-1-2-3
  VictorSP motor0LeftBack =  new VictorSP(0);
  VictorSP motor1LeftFront =  new VictorSP(1);
  VictorSP motor2RightBack =  new VictorSP(2);
  VictorSP motor3RightFront =  new VictorSP(3);

  MotorControllerGroup left = new MotorControllerGroup(motor0LeftBack, motor1LeftFront);
  MotorControllerGroup right = new MotorControllerGroup(motor2RightBack, motor3RightFront);

  DifferentialDrive myRobot = new DifferentialDrive(left,right);

  double chassisLimit = 1;
  //Gyro Setup
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  static final double maxDegrees = 11;
  double rollAngleDegrees;
  boolean autoBalanceYMode;
  
  // Relative encoder, başlangıç pozisyonunu not ediyoruz.
  private double enc2ZeroVal;
  // Integral partı için errorleri topluyor.
  double errorSum1 = 0;
  double errorSum2 = 0;

  // Kolun gitmek istediği açı
  double targetPos1 = -5;
  double targetPos2;

  // Integral hesabı için bir önceki tick'teki zaman
  double lastTimeStamp1 = 0;
  double lastTimeStamp2 = 0;

  // Derivative partı için bir önceki tick'teki error değeri
  double lastError1 = 0;
  double lastError2 = 0;

  double error1;
  double error2;


  double addVariable = 0;
  // Derivate partı için error eğimi.
  double errorRate1 = 0;
  double errorRate2 = 0;

  double VoltageVal;
  // for chassis drive

  @Override
  public void robotInit() {
    // Radio'ya bağlanma kodu ...




    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    // Pinler ile initialize ediyor.
    xboxController = new XboxController(0);
    encoderJoint1 = new DutyCycleEncoder(3);
    // CAN ID: 1, Intake için
    intake = new CANSparkMax(1, MotorType.kBrushless);

    // Gearbox motorları MotorJoint1
    VictorSP motorJoint1a = new VictorSP(4); 
    VictorSP motorJoint1b = new VictorSP(5); 
    // Motor Joint 1'i oluşturan motorları grupluyor.
    motorJoint1 = new MotorControllerGroup(motorJoint1a, motorJoint1b);

    // Motor CAN ID:2 olarak set, bu ikinci joint motoru. DeviceID'leri kontrol edelim.
    motorJoint2 = new CANSparkMax(2, MotorType.kBrushless); 
    // Encoder değerlerini alacak bir obje yaratıyor.
    encoderJoint2 = motorJoint2.getEncoder();
    // Tick'ten pozisyona gitmek için tick sayısı 42
    encoderJoint2.setPositionConversionFactor(42);
    
    // Her Çalıştığında yeniden 0'lıyor aldığı referans aksisini 


    // Switchler initialize ediliyor.
    Joint2LowerSwitch = new DigitalInput(2);
    StopSwitch = new DigitalInput(1);
  }



  @Override
  public void teleopInit() {
    // Neo motorunun içindeki encoder'e erişerek pozisyon değerine erişiyor.
    enc2ZeroVal = encoderJoint2.getPosition();
    motorJoint2.getInverted();
    myRobot.setSafetyEnabled(true);
    intakeOpen = false;
    quarter = false;
    //motorJoint2.enableVoltageCompensation(8);
    //motorJoint2.setSmartCurrentLimit(28);
    //motorJoint1a.enableVoltageCompensation(true);
  }

  @Override
  public void autonomousInit() {
    myRobot.setSafetyEnabled(false);
    myTimer = new Timer();
    myTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
      //System.out.println("1");
      //myRobot.tankDrive(-0.5, 0.5);
      if(myTimer.get() < 0.25) {
        motorJoint1.set(0.65);
        intake.set(-0.4);
      }  
      else if(myTimer.get() < 3){
        motorJoint1.set(0.24);
        intake.set(-0.4);
      }
      else{
        motorJoint1.set(0.05);
      }

      if(myTimer.get()  > 2 && myTimer.get()  < 2.5){
        intake.set(0.3);
      }

      else{
        intake.set(0);
      }

      //For Alliance Area Exit
      if (myTimer.get() > 5 && myTimer.get() < 6.5){
        myRobot.tankDrive(-0.9, 0.9);  
      }
      if (myTimer.get() > 6.2){
        myRobot.tankDrive(0, 0); 
      }

    } 
  @Override
  public void teleopPeriodic() {


    // Absolute encoder yatay posizyonda 293 değer veriyor. Bundan dolayı 293 çıkarak 0 derece üzeriden hesaplama yapıyoruz.
    double enc1Ang = (encoderJoint1.getAbsolutePosition() * 360)-115; //sonradan değiştir
    // Tork ve RPM değeri bulunabilir. İşe yarar mı bakılsın. 
    // 0.014 - 0.015 gibi bir değerdeydi en son
    double kp2 = 0.0000000008;

    //DEBUG
    System.out.println("enc1Ang: " + enc1Ang);
    //System.out.println("Yaw" + gyro.getYaw());
    //System.out.println("Roll" + gyro.getRoll());
    //System.out.println("Pitch" + gyro.getPitch());

    double error1 = targetPos1 - enc1Ang;
      
      
      // INTEGRAL
      // kI1 is the constant for the Integral part. We will be calculating the offset in the error value continuously and increasing the motor power proportionally.
      // 5 ile çarptım
    double kI1 = 0.00065;
    double dt1 = Timer.getFPGATimestamp() - lastTimeStamp1;

    errorSum1 += error1 * (dt1);  
    
    System.out.println("dt1: " + dt1);
    System.out.println("errorSum1: " + errorSum1);
  // DERIVATIVE
    double kD1 = 0;
    double errorRate1 = (error1 - lastError1) / dt1;
  // MOTOR OUTPUT
  // error'u çıkardım
    double p1 = (( kp2 * error1 ) + (kI1 * errorSum1));



    


   // XBOX RB butonu
   //  High Goal Koymadan Position
      if(xboxController.getXButton() && intakeOpen == false){
      intakeOpen = true;
      intake.set(-0.35);
     
    }
      else if (xboxController.getXButton() && intakeOpen == true){
        intakeOpen = false;
        intake.set(0);
      }
    

    // Intake Position
    if(xboxController.getBButton() && intakeOpen == false){
      intakeOpen = true;
      intake.set(0.35);
    }
    else if (xboxController.getBButton() && intakeOpen == true){
      intakeOpen = false;
      intake.set(0);
    }
    
   
    System.out.println(enc1Ang);
/* 

    if(xboxController.getAButton()){
      
      // PROPORTIONAL
      //theorethical 42, given 73  
      //targetPos1 = -90;

      
      //double p1 = (kp1 * error1)+(kI1 * errorSum2)+(kD1 * errorRate1);
     
    }
    
    if (xboxController.getYButton()) {
      
      // PROPORTIONAL
      //theorethical 42, given 73  
      //targetPos1 = -35;

      
      
    }
    lastTimeStamp1 = Timer.getFPGATimestamp();
    lastError1 = error1;
    System.out.println("power"+p1);

    System.out.println(motorJoint2.get());
    // açıya bağlı bir şekilde limiti arttır.
    if ( p1>0.40) {
      p1 = (0.4);
    }
  
    motorJoint1.set(-p1);
    
    
    System.out.println("p1:" + p1);
    
*/

    if(xboxController.getRightBumper()){
      motorJoint1.set(0.30);
    } 
    else if (xboxController.getLeftBumper()){
      motorJoint1.set(-0.10);
    }
    else {motorJoint1.set(0.1);}

   
  
  
/* 
    if(xboxController.getStartButton()){

      motorJoint2.set(0.32);
    } */
    
    if(xboxController.getBackButton()){
     
      quarter = true;

      chassisLimit = 0.5;
    }
    else if(xboxController.getStartButton()){
      quarter = false;

      chassisLimit = 1;
    }




     
    if(xboxController.getRightX() > 0.1 || xboxController.getRightX() < -0.1 ){
      myRobot.tankDrive(xboxController.getRightX() * chassisLimit, xboxController.getRightX()*chassisLimit);
      System.out.println("1");
    }
    else{
      myRobot.tankDrive(-xboxController.getLeftY()*chassisLimit, xboxController.getLeftY()*chassisLimit);
      System.out.println("3");
    }
    



    }
  }
