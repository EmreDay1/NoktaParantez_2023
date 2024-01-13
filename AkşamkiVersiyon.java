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



  //Chassis motorları 0-1-2-3
  VictorSP motor0LeftBack =  new VictorSP(0);
  VictorSP motor1LeftFront =  new VictorSP(1);
  VictorSP motor2RightBack =  new VictorSP(2);
  VictorSP motor3RightFront =  new VictorSP(3);

  MotorControllerGroup left = new MotorControllerGroup(motor0LeftBack, motor1LeftFront);
  MotorControllerGroup right = new MotorControllerGroup(motor2RightBack, motor3RightFront);

  DifferentialDrive myRobot = new DifferentialDrive(left,right);

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
  double targetPos1;
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
      if(myTimer.get() < 0.2) {
        motorJoint2.set(-0.65);
        intake.set(-0.4);
      }  
      else if(myTimer.get() < 3){
        motorJoint2.set(-0.20 );
        intake.set(-0.4);
      }
      else{
        motorJoint2.set(-0.07);
      }

      if(myTimer.get()  > 2 && myTimer.get()  < 2.5){
        intake.set(0.3);
      }

      else{
        intake.set(0);
      }

      //For Alliance Area Exit
      if (myTimer.get() > 5 && myTimer.get() < 6){
        myRobot.tankDrive(-0.9, 0.9);  
      }
      if (myTimer.get() > 6.2){
        myRobot.tankDrive(0, 0); 
      }
   
  

  @Override
  public void teleopPeriodic() {


    // HESAPLAMAYI OBJECT LEVELDA YAP HER SANIYE YAPMANA GEREK YOK.
    // Sayılar dişli sayılarını temsil ediyor. 56 Aktarılan büyük, bölümdeki küçük gearlar.
    double gearboxRatio = (56*56*56)/(24*24*14);
    // gearboxRatio'u RPM'e çeviriyor.
    double gearboxRPM = 1/gearboxRatio;
    // Joint 2 Neo Motorunun tick sayısı.
    double encoder2CPR = 42.0;
    // 
    double enc2Tick = encoderJoint2.getPosition() - enc2ZeroVal;
    // NEDEN 100'le Çarpıldığı anlaşılmadı. Bilmiyoruz ancak bildiğimiz bir şey varsa o da 360 derecenin tick'i açıya çevirmek için konduğudur.
    // 100'i sildim. Yüzsüzüm.
    double enc2Ang = (((enc2Tick/encoder2CPR) * gearboxRPM) * 360);
    // Absolute encoder yatay posizyonda 293 değer veriyor. Bundan dolayı 293 çıkarak 0 derece üzeriden hesaplama yapıyoruz.
    double enc1Ang = (encoderJoint1.getAbsolutePosition() * 360); //sonradan değiştir
    // Tork ve RPM değeri bulunabilir. İşe yarar mı bakılsın. 
    double kp1 = 0.017;
    double kp2 = 0.013;
    double kp1Manual = 0.15;
    double kp2Manual = 0.2;
    //DEBUG
    System.out.println("enc1Ang: " + enc1Ang);
    


   // XBOX RB butonu
   //  High Goal Koymadan Position
      if(xboxController.getXButton()){
      intake.set(-0.3);
     
    }

    // Intake Position
    else if(xboxController.getBButton()){
      intake.set(0.3);
    }
    System.out.println(enc1Ang);


    if(xboxController.getAButton()){
      
      // PROPORTIONAL
      //theorethical 42, given 73  
      targetPos1 = 270;

      double error1 = targetPos1 - enc1Ang;
      
      
      // INTEGRAL
      // kI1 is the constant for the Integral part. We will be calculating the offset in the error value continuously and increasing the motor power proportionally.
      // 5 ile çarptım
      double kI1 = 0.00000000000000000006;
      double dt1 = Timer.getFPGATimestamp() - lastTimeStamp1;
      errorSum1 += error1 * dt1;
      System.out.println("dt1: " + dt1);
      System.out.println("errorSum1: " + errorSum1);
  // DERIVATIVE
      double kD1 = 0;
      double errorRate1 = (error1 - lastError1) / dt1;
  // MOTOR OUTPUT
  // error'u çıkardım
      double p1 = -(( kp2 * error1 ) + (kI1 * errorSum1));
      //double p1 = (kp1 * error1)+(kI1 * errorSum2)+(kD1 * errorRate1);
      motorJoint2.set(p1);
      lastTimeStamp1 = Timer.getFPGATimestamp();
      lastError1 = error1;
      System.out.println("power"+p1);

      System.out.println(motorJoint2.get());
      //myRobot.arcadeDrive(xboxController.getLeftX()/3,xboxController.getLeftY()/3);




    }  
    
    




    if(xboxController.getRightBumper()){

      motorJoint2.set(-0.28);
    } 

    else if (xboxController.getYButton()){
      motorJoint2.set(0);
    }
    

    if(xboxController.getStartButton()){

      motorJoint2.set(0.32);
    }
    /* 
    else if (xboxController.getYButton()){
      
      motorJoint2.set(0.235);
    }
    */
    if(xboxController.getBackButton()){
         
      motorJoint2.set(-0.40);
      
    }




    //myRobot.tankDrive(kp1Manual, kp2Manual);
     
    if(xboxController.getRightX() > 0.1 || xboxController.getRightX() < -0.1 ){
      myRobot.tankDrive(xboxController.getRightX(), xboxController.getRightX());
      System.out.println("1");
    }
    else{
      myRobot.tankDrive(-xboxController.getLeftY(), xboxController.getLeftY());
      System.out.println("3");
    }
    

    //myRobot.arcadeDrive(xboxController.getLeftX(),xboxController.getLeftY());
/* 
    if (xboxController.getLeftStickButton()){

      intake.set(0.5);
    }
    else if (xboxController.getRightStickButton()){
      intake.set(-0.5);
    }
    else {
      intake.set(0);
    } 
*/
    if(Joint2LowerSwitch.get() == false){
      enc2Ang = 0;
      //motorJoint2.set(0);
      //System.exit(2);
    }
    if(StopSwitch.get() == false){
      System.exit(6431);
    } 


    }
  }
