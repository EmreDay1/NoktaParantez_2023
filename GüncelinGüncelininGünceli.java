package frc.robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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
import pabeles.concurrency.IntOperatorTask.Min;
import edu.wpi.first.wpilibj.PWM;
import java.lang.Math;
import java.util.concurrent.DelayQueue;

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

  
  // Relative encoder, başlangıç pozisyonunu not ediyoruz.
  private double enc2ZeroVal;
  // Integral partı için errorleri topluyor.
  double errorSum1 = 0;
  double errorSum2 = 0;

  // Kolun gitmek istediği açı
  double targetPos1;
  double targetPos2;

  // Integral hesabı için bir önceki tick'teki zaman
  double CurrentTime = 0;
  double LastTime = 0;

  // Derivative partı için bir önceki tick'teki error değeri
  double lastError1 = 0;
  double lastError2 = 0;

  double error1;
  double error2;

  // Derivate partı için error eğimi.
  double errorRate1 = 0;
  double errorRate2 = 0;

  // for chassis drive

  // Variables containing the angular velocity
  double TheorethicalAngVel = 0;
  double EmpiricalAngVel = 0;

  double enc2Ang;
  double Lastenc2Ang;

  double changeInAngle = 2;
  //Double[20] = [];

  double myVolt;

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
    Joint2LowerSwitch = new DigitalInput(1);
    StopSwitch = new DigitalInput(3);
  }

  @Override
  public void teleopInit() {
    // Neo motorunun içindeki encoder'e erişerek pozisyon değerine erişiyor.
    enc2ZeroVal = encoderJoint2.getPosition();
  }

  @Override
  public void teleopPeriodic() {



    myRobot.arcadeDrive(xboxController.getLeftY(),xboxController.getLeftX());
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
    enc2Ang = (((enc2Tick/encoder2CPR) * gearboxRPM) * 360);

    // Absolute encoder yatay posizyonda 293 değer veriyor. Bundan dolayı 293 çıkarak 0 derece üzeriden hesaplama yapıyoruz.
    double enc1Ang = (encoderJoint1.getAbsolutePosition() * 360) - 293; //sonradan değiştir
    // Tork ve RPM değeri bulunabilir. İşe yarar mı bakılsın. 
    double kp1 = 0.012;
    double kp2 = 0.15;
    double kp1Manual = 0.2;
    double kp2Manual = 0.2;
    //DEBUG
    System.out.println("enc1Ang: " + enc1Ang);
    


    targetPos2 = enc2Ang;

   // XBOX RB butonu
    if(xboxController.getRightBumper()){
      //motorJoint1.set(0.3);
      targetPos1 = enc1Ang + 2;
      double error1 = targetPos1 - enc1Ang;
      double p1 = (kp1Manual* error1);
      motorJoint1.set(p1);
      
     
    }
    else if(xboxController.getLeftBumper()){
      //motorJoint1.set(-0.3);
      targetPos1 = enc1Ang - changeInAngle;       
      double error1 = targetPos1 - enc1Ang;
      double p1 = (kp1Manual* error1);
      motorJoint1.set(p1 * (1+Math.sin(Math.toRadians(enc2Ang))));    }

    else {
      motorJoint1.set(0.1);
    }

    System.out.println("enc2Angle: " + enc2Ang);
    // 2 is Current, 1 is Last.
    Timer myTimer = new Timer();
    CurrentTime = myTimer.getFPGATimestamp();



    //TheorethicalAngVel = (changeInAngle /(LastTime-CurrentTime));

    
    if(xboxController.getAButton()){
      targetPos2 = enc2Ang - changeInAngle; 
      double error2 = targetPos2 - enc2Ang;
      double p2 = (kp2Manual * error2);
      motorJoint2.set(p2);
      LastTime = CurrentTime;
      CurrentTime = myTimer.getFPGATimestamp();
      
    }

    else if(xboxController.getYButton()){
      targetPos2 = enc2Ang + changeInAngle; 
      double error2 = targetPos2 - enc2Ang;
      double p2 = (kp2Manual* error2);
      motorJoint2.set(p2);
      CurrentTime = myTimer.getFPGATimestamp();
    }
    else {
      /*
      double pV = 0.05 + ((TheorethicalAngVel-EmpiricalAngVel));
      if(pV <= -0.1 || pV>0.1){
        motorJoint2.set(pV);

      }
      System.out.println(pV);
      EmpiricalAngVel = (enc2Ang-Lastenc2Ang)/(CurrentTime-LastTime);
      */

      EmpiricalAngVel = (enc2Ang-Lastenc2Ang)/(CurrentTime-LastTime);
      CurrentTime = myTimer.getFPGATimestamp();

      double a1 = 0.928;
      double a2 = 0.83;
      double M2 = 0.45;
      double MIntake = 0; //0.425
      double LIntake = 0; // 0.225
      double torque = ((Math.cos(enc2Ang) + ((a2/2) * Math.cos(enc2Ang))) * M2 * 9.8) +  (a1* Math.cos(enc1Ang) + a2 * Math.cos(enc2Ang)) * (MIntake * 9.8) + (a1 * Math.cos(enc1Ang) + a2* Math.cos(enc2Ang)+ (LIntake/2) * (MIntake * 9.8));

      System.out.println("torque: " + torque);
      double mySpeed = 0.31;
      myVolt = (torque * (2 * Math.PI)  * mySpeed) / (60 * 40); // 40 = Const. Ampere


      motorJoint2.setVoltage(myVolt);

      // SAFETY don't allow values greater than some and less than some.
     
          }

    if(xboxController.getXButton()){
      intake.set(0.3);
    }
    else if(xboxController.getBButton()){
      intake.set(-0.3);
    }  
    else{
      intake.set(0);
    }
    
/* 
    if(xboxController.getAButton()){
 // PROPORTIONAL
      targetPos1 = enc1Ang + 10;
      double error1 = targetPos1 - enc1Ang;

      // INTEGRAL

      // kI1 is the constant for the Integral part. We will be calculating the offset in the error value continuously and increasing the motor power proportionally.
      double kI1 = 0.01;
      double dt1 = Timer.getFPGATimestamp() - lastTimeStamp1;
      errorSum1 += error1 * dt1;

      
  // DERIVATIVE
      double kD1 = 0;
      double errorRate1 = (error1 - lastError1) / dt1;


  // MOTOR OUTPUT
      double p1 = kp1 * error1;
      //double p1 = (kp1 * error1)+(kI1 * errorSum2)+(kD1 * errorRate1);
      motorJoint1.set(p1);
      lastTimeStamp1 = Timer.getFPGATimestamp();
      lastError1 = error1;
/* 
      System.out.println("Encoder1 DutyCycle: " + enc1Ang);
      System.out.println("P-Value: "+ p1);
      System.out.println("Error-Value: "+ error1);
    }  */
    
    

    /* 
    if(xboxController.getYButton()){

      

    //PROPORTIONAL
      targetPos2 = enc2Ang + 10;
      double error2 = targetPos2 - enc2Ang;
    // INTEGRAL
      double kI2 = 0;
      double dt2 = Timer.getFPGATimestamp() - lastTimeStamp2;
      errorSum2 += error2 * dt2;


    // DERIVATIVE
      double kD2 = 0;
      double errorRate2 = (error2- lastError1) / dt2;

    // MOTOR OUTPUT
      double p2 = (kp2 * error2)+(kI2 * errorSum2)+(kD2 * errorRate1);

     // motorJoint2.set(p2);
      lastTimeStamp2 = Timer.getFPGATimestamp();
      lastError2 = error2;
/* 
      System.out.println("Encoder2 DutyCycle: " + enc2Ang);
      System.out.println("P2-Value: " + p2);
      System.out.println("Error2-Value: " + error2);
       }  */


/* 
    if(xboxController.getXButton()){
      targetPos1 += 5;
      targetPos2 += 5;
      System.out.println("Decreased TargetPos1 to: " + targetPos1);
      System.out.println("Decreased TargetPos2 to: " + targetPos2);

    }
 
    if(xboxController.getYButton()){
      targetPos1 += 5;
      targetPos2 += 5;
      System.out.println("Decreased TargetPos1 to: " + targetPos1);
      System.out.println("Decreased TargetPos2 to: " + targetPos2);
    }

    if(limSwitch.get() == false){
      System.exit(0);
    }
*/


/* 
    if(Joint2LowerSwitch.get() == false){
      motorJoint2.set(0);
      System.exit(2);
    }

    if(StopSwitch.get() == false){
      System.exit(31);
    }
*/


    motor0LeftBack.setVoltage(kp2Manual);
    }
    public double feedForwardFeedback(double enc2Ang, double kF){

      // Take the angle deviation from the start position of the second encoder.
      // take the absolute value of the angle deviation and multiply with a constant
      // Add the value to the motor.

      // Neden 90 verdik?
      double FeedForwardPush = (90-Math.abs(enc2Ang)) * kF;
      return (FeedForwardPush);
    }

  }
