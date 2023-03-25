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
    encoderJoint1 = new DutyCycleEncoder(0);
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







      //For Gyro Balancing Bu kod commentlemezsen emergency verir !!!!!!!!
      //if (myTimer.get() > 5 && myTimer.get() < 7.5){
        //myRobot.tankDrive(-1, 1);
      //}
      /*if (myTimer.get() > 1){
        //myRobot.tankDrive(0, 0); 
        rollAngleDegrees = gyro.getRoll();
        System.out.println(rollAngleDegrees);

          if ( !autoBalanceYMode && 
          (Math.abs(rollAngleDegrees) >= 
          Math.abs(maxDegrees))) {
                autoBalanceYMode = true;
          }
          else if ( autoBalanceYMode && 
            (Math.abs(rollAngleDegrees) <= 
            Math.abs(maxDegrees))) {
                autoBalanceYMode = false;
            }
          if ( autoBalanceYMode = true ) {
            double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
            double yAxisRate = Math.sin(rollAngleRadians) * -1;
            myRobot.tankDrive(-yAxisRate, yAxisRate);  
          }
          else{
            myRobot.tankDrive(0, 0);
            return;
          }
      
      }*/
    } 
    

      /*if (myTimer.get() < 6.1){ 
        myRobot.tankDrive(-1, 1);  
      }
      if (myTimer.get() > 6.1){
        myRobot.tankDrive(0, 0);*/
        //rollAngleDegrees = gyro.getRoll();
        //System.out.println(rollAngleDegrees); 
        
        
        
        
        
        
        
        
        
        
        
        /* 
        //System.out.println("timer: " + myTimer);
        rollAngleDegrees = gyro.getRoll();
        System.out.println(rollAngleDegrees);
        while (maxDegrees > rollAngleDegrees){ 

          if ( !autoBalanceYMode && 
          (Math.abs(rollAngleDegrees) >= 
          Math.abs(maxDegrees))) {
                autoBalanceYMode = true;
          }
          else if ( autoBalanceYMode && 
            (Math.abs(rollAngleDegrees) <= 
            Math.abs(maxDegrees))) {
                autoBalanceYMode = false;
            }
        if ( autoBalanceYMode = true ) {
          double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
          double yAxisRate = Math.sin(rollAngleRadians) * -1;
          myRobot.tankDrive(-yAxisRate, yAxisRate);  
        }
        else{
          myRobot.tankDrive(0, 0);
          return;
        }

      //Timer.delay(0.05);
      
    }*/
   
  

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
    double enc1Ang = (encoderJoint1.getAbsolutePosition() * 360) + 20; //sonradan değiştir
    // Tork ve RPM değeri bulunabilir. İşe yarar mı bakılsın. 
    double kp1 = 0.017;
    double kp2 = 0.011;
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
    else if (xboxController.getYButton()){
      intake.set(0);
    }


    if(xboxController.getAButton()){
      /* 
      // PROPORTIONAL
      //theorethical 42, given 73
      targetPos1 = 60;

      double error1 = targetPos1 - enc1Ang;
      System.out.println(enc1Ang);
      
      // INTEGRAL
      // kI1 is the constant for the Integral part. We will be calculating the offset in the error value continuously and increasing the motor power proportionally.
      // 5 ile çarptım
      double kI1 = 0.00000006;
      double dt1 = Timer.getFPGATimestamp() - lastTimeStamp1;
      errorSum1 += error1 * dt1;
      System.out.println("dt1: " + dt1);
      System.out.println("errorSum1: " + errorSum1);
  // DERIVATIVE
      double kD1 = 0;
      double errorRate1 = (error1 - lastError1) / dt1;
  // MOTOR OUTPUT
  // error'u çıkardım
      double p1 = ( kp1 * error1 ) + (kI1 * errorSum1);
      //double p1 = (kp1 * error1)+(kI1 * errorSum2)+(kD1 * errorRate1);
      motorJoint2.set(p1);
      lastTimeStamp1 = Timer.getFPGATimestamp();
      lastError1 = error1;
      System.out.println("power"+p1);

      //myRobot.arcadeDrive(xboxController.getLeftX()/3,xboxController.getLeftY()/3);

      //motorJoint2.set(0.25); */

      motorJoint2.set(0);


    }  
    
    

/* 
    // Actual position will be a little less than the desired
    if(xboxController.getYButton()){
      
    //PROPORTIONAL
    // subtracted 10 degrees
    //theorethical  -98.4, given -120
      targetPos2 = 98;

      double error2 = targetPos2 - enc1Ang;


    // INTEGRAL
      double kI2 = 0.000001;
      double dt2 = Timer.getFPGATimestamp() - lastTimeStamp2;
      errorSum2 += error2 * dt2;
    // DERIVATIVE
      double kD2 = 0;
      double errorRate2 = (error2- lastError1) / dt2;
    // MOTOR OUTPUT
    //+(Math.sin(Math.toRadians(-enc2Ang))/10)
      double p2 = (kp2 * error2)+(kI2 * errorSum2);
      motorJoint2.set(p2);
      lastTimeStamp2 = Timer.getFPGATimestamp();
      lastError2 = error2;
      myRobot.arcadeDrive(xboxController.getLeftX()/3,xboxController.getLeftY()/3);

 
       } 
*/


    if(xboxController.getRightBumper()){
/*        // PROPORTIONAL
       // Theorethical 42.9 given value 68
       targetPos1 = 58;
       double error1 = targetPos1 - enc1Ang;
       // INTEGRAL
       // kI1 is the constant for the Integral part. We will be calculating the offset in the error value continuously and increasing the motor power proportionally.
       // 5 ile çarptım
       double kI1 = 0.0001;
       double dt1 = Timer.getFPGATimestamp() - lastTimeStamp1;
       errorSum1 += error1 * dt1;
       
   // DERIVATIVE
       double kD1 = 0;
       double errorRate1 = (error1 - lastError1) / dt1;
   // MOTOR OUTPUT
       double p1 = ( kp1 * error1 ) + (kI1 * errorSum1);
       //double p1 = (kp1 * error1)+(kI1 * errorSum2)+(kD1 * errorRate1);
       motorJoint1.set(p1/1.5);
       lastTimeStamp1 = Timer.getFPGATimestamp();
       lastError1 = error1;
       myRobot.arcadeDrive(xboxController.getLeftX()/3,xboxController.getLeftY()/3);

*/
      motorJoint2.set(-0.27 + addVariable);
    } 
    else if (xboxController.getLeftBumper()){
      motorJoint2.set(-0.15);
    }
  


    /*if(xboxController.getBButton()){
            
    //PROPORTIONAL
    //Needed -47, given -67
      targetPos2 = -57;

      double error2 = targetPos2 - enc2Ang;


    // INTEGRAL
      double kI2 = 0.00001;
      double dt2 = Timer.getFPGATimestamp() - lastTimeStamp2;
      errorSum2 += error2 * dt2;
    // DERIVATIVE
      double kD2 = 0;
      double errorRate2 = (error2- lastError1) / dt2;
    // MOTOR OUTPUT
    //+(Math.sin(Math.toRadians(-enc2Ang))/10)
      double p2 = (kp2 * error2)+(kI2 * errorSum2);
      motorJoint2.set(p2/2.5);
      lastTimeStamp2 = Timer.getFPGATimestamp();
      lastError2 = error2;
      myRobot.arcadeDrive(xboxController.getLeftX()/3,xboxController.getLeftY()/3);

      
    }
    if(xboxController.getStartButton()){
       // PROPORTIONAL

       // Theorethical 55, given value 70
       targetPos1 = 70;
       double error1 = targetPos1 - enc1Ang;
       // INTEGRAL
       // kI1 is the constant for the Integral part. We will be calculating the offset in the error value continuously and increasing the motor power proportionally.
       // 5 ile çarptım
       double kI1 = 0.0001;
       double dt1 = Timer.getFPGATimestamp() - lastTimeStamp1;
       errorSum1 += error1 * dt1;
       
   // DERIVATIVE
       double kD1 = 0;
       double errorRate1 = (error1 - lastError1) / dt1;
   // MOTOR OUTPUT
       double p1 = ( kp1 * error1 ) + (kI1 * errorSum1);
       //double p1 = (kp1 * error1)+(kI1 * errorSum2)+(kD1 * errorRate1);
       motorJoint1.set(p1/1.5);
       lastTimeStamp1 = Timer.getFPGATimestamp();
       lastError1 = error1;
       myRobot.arcadeDrive(xboxController.getLeftX()/3,xboxController.getLeftY()/3);


    }*/

    if(xboxController.getStartButton()){
      addVariable += 0.001;
    }
    if(xboxController.getBackButton()){
         /* 
    //PROPORTIONAL
      targetPos2 = 0;

      double error2 = targetPos2 - enc2Ang;


    // INTEGRAL
      double kI2 = 0.00001;
      double dt2 = Timer.getFPGATimestamp() - lastTimeStamp2;
      errorSum2 += error2 * dt2;
    // DERIVATIVE
      double kD2 = 0;
      double errorRate2 = (error2- lastError1) / dt2;
    // MOTOR OUTPUT
    //+(Math.sin(Math.toRadians(-enc2Ang))/10)
      double p2 = (kp2 * error2)+(kI2 * errorSum2);
      motorJoint2.set(p2/2.5);
      motorJoint2.setVoltage(kp2Manual);
      lastTimeStamp2 = Timer.getFPGATimestamp();
      lastError2 = error2;
      myRobot.arcadeDrive(xboxController.getLeftX()/3,xboxController.getLeftY()/3);
*/
      addVariable -= 0.001;
      
      
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
