
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "mcc_generated_files/examples/i2c2_master_example.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define wheel_radius   32
#define btw_2wheels    69.5
#define hSTEP_RIGHT LATFbits.LATF6
#define hDIR_RIGHT  LATFbits.LATF7
#define hMS1_RIGHT  LATFbits.LATF4
#define hMS2_RIGHT  LATFbits.LATF5

#define hSTEP_LEFT LATFbits.LATF2
#define hDIR_LEFT  LATFbits.LATF3
#define hMS1_LEFT  LATFbits.LATF0
#define hMS2_LEFT  LATFbits.LATF1

#define hSTEP_DP   LATBbits.LATB1
#define hDIR_DP    LATBbits.LATB0
#define hMS1_DP    LATBbits.LATB3
#define hMS2_DP    LATBbits.LATB2

#define HIGH   1
#define LOW    0
#define T_MAX      1000  

#define totalTimeMax  100000
#define totalTimeMin  0
#define OMEGA_MAX     7
//#define PID_CLASSIC
//#define PID_SOME_OVER
#define PID_NO_OVER
#ifdef PID_CLASSIC
    #define Kp_Omega  0.09
    #define Ki_Omega  0.025
    #define Kd_Omega  0.078
#endif

#ifdef PID_SOME_OVER
    #define Kp_Omega  0.05     // 2 5 10
    #define Ki_Omega  0.014     // 0.01 0.1 0.05
    #define Kd_Omega  0.117
#endif

#ifdef PID_NO_OVER
    #define Kp_Omega  0.142
    #define Ki_Omega  0
    #define Kd_Omega  0
#endif

#define PID_OMEGA_MAX   8
#define PID_OMEGA_MIN  -8

#define PI              3.14159
#define ROBOT_RUN       1
#define ROBOT_STOP      0

// initialize interrupt 
unsigned int t_left  = T_MAX, cntLeft  = 0;      // delay for LEFT pulse
unsigned int t_right = T_MAX, cntRight = 0;     // delay for Right pulse
unsigned int t_dp = T_MAX, cntDp = 0;          // delay for DP pulse
uint16_t totalTime = totalTimeMin, cntTime = 0;
unsigned char flag_stop = ROBOT_STOP;

// initialize distance sensor
uint8_t shift = 0, shift_one = 0;
float  distance_raw[2] = {0,0};
float  distance_raw_one[2] = {0,0};
float  DistanceValueTop    = 0;
float  DistanceValueBottom = 0;

// initialize variable PID
float  errorOmega=0, errorOmega_pre=0;
double errorOmega_sum = 0;
float  PID_Omega=0;

// initialize manual control
unsigned char flagManual = 0;
float velManual   = 0, velManual_t = 0;
float omegaManual = 0;
int   cntManual = 0;

// initialize auto line control
unsigned char flagALine = 0, cntALine = 0;
unsigned int  length_ALine = 0;
int direct_ALine = 0;
float velALine = 0, omegaALine = 0, timeALine = 0;

// initialize auto circle control
unsigned char flagACircle = 0, cntACircle = 0;
unsigned int  angle_ACircle = 0;
float velACircle = 0, omegaACircle = 0, valRadius_AC = 0, timeACircle = 0;

// initialize camera control
unsigned char   cntCamera = 0; 
unsigned char flagCamera = 0;
float valRadius_TL = 0, valVLength_TL = 0, valTime_TL = 0;
unsigned int sum_frame = 0, delay_frame = 0, cnt_frame = 0;
float angle_move = 0, omega_TL = 0, velocity_TL = 30, time_move = 0;

//initialize system
char buffer_sys[50], in_ramp = 1;
unsigned char cnt_sys = 0;
float velocity_sys = 0, omega_sys = 0;

void putch(char value)
{
   while(!EUSART2_is_tx_ready());
   EUSART2_Write(value);
}

void T1_ROBOT_ISR()
{
    if (t_left != T_MAX){
        if (cntLeft < (float)(t_left/2)) cntLeft++;   
        else{
            hSTEP_LEFT = !hSTEP_LEFT;
            cntLeft = 0;
        }
    }
    if (t_right != T_MAX){
        if (cntRight < (float)(t_right/2)) cntRight++;
        else {
            hSTEP_RIGHT = !hSTEP_RIGHT;
            cntRight = 0;
        } 
    }
    if (t_dp != T_MAX){
        if (cntDp < (float)(t_dp/8)) cntDp++;
        else {
            hSTEP_DP = !hSTEP_DP;
            cntDp = 0;
        } 
    }     
    if(cntTime < totalTime)   cntTime = cntTime + 1;
    else{
        flag_stop = ROBOT_STOP;
        omega_sys = 0;
        velocity_sys = 0;
        t_left  = T_MAX;
        t_right = T_MAX;
    }
}

void KE(float Omega_body, float Velocity_body){
    float OmegaLeft = 0;
    float OmegaRight = 0;
    // 1ms ngat timer thi OmegaLeft MAX = 15,7 rad/s
    OmegaLeft  = Velocity_body/wheel_radius + Omega_body*(btw_2wheels/wheel_radius);
    OmegaRight = Velocity_body/wheel_radius -  Omega_body*(btw_2wheels/wheel_radius); 
    /*
     run_forward --> DIR_RIGHT = HIGH; DIR_LEFT = LOW;
     run_backward --> DIR_RIGHT = LOW; DIR_LEFT = HIGH;   
     */ 
    if(OmegaLeft < 0){
        OmegaLeft  = -OmegaLeft;
        hDIR_RIGHT = 0;       
    }
    else hDIR_RIGHT = 1; 
    
    if(OmegaRight < 0){
        OmegaRight = -OmegaRight;
        hDIR_LEFT = 1;
    }
    else hDIR_LEFT = 0;
    
    if (OmegaLeft == 0)     t_left  = T_MAX;   // Banh xe trai ngung quay
    else                    t_left  = (unsigned int)(31.416/OmegaLeft)/2;
    if (OmegaRight == 0)    t_right = T_MAX;   // Banh xe phai ngung quay
    else                    t_right = (unsigned int)((31.416+2)/OmegaRight)/2;
}

void ramp_step_function(float velocity, float radius, float total_time)
{
    float omega_ramp = 0, velocity_ramp = velocity_sys;
    for(int i = 0; i < (total_time*0.2); i++) {
        __delay_ms(1);
        if((velocity_sys == 0) && (velocity != 0)) {
            if(velocity_ramp != velocity) velocity_ramp = velocity_ramp + (0.05*velocity);
        }
        else if((velocity_sys != 0) && (velocity == 0)) {
            if(velocity_ramp != 0) velocity_ramp -= (0.05*velocity_sys);
        }
        if(radius != 0) {
            omega_ramp = velocity_ramp/radius;
        }
        KE(omega_ramp,velocity_ramp);
    }
    in_ramp = 0;
}

float distance_sensor_bottom(){    
 //   __delay_us(800);       
    // Distance[11:4]
    shift = I2C2_Read1ByteRegister(0x80 >>1,0x35);
    distance_raw[0] = I2C2_Read1ByteRegister(0x80 >> 1,0x5E);
    // Distance[3:0]
    distance_raw[1] = I2C2_Read1ByteRegister(0x80 >> 1,0x5F);         
    return ((distance_raw[0] * 16 + distance_raw[1])/16/4);
}
float distance_sensor_top(){
    // Open power, delay ~ 800us, 
   // __delay_us(500);
    // Keep PIN RC1-GPIO HIGH, delay ~ 800us
   // GPIO1 = HIGH;    
 //   __delay_us(800);       
    // Distance[11:4]
    shift_one = I2C1_Read1ByteRegister(0x80 >>1,0x35);
    distance_raw_one[0] = I2C1_Read1ByteRegister(0x80 >>1,0x5E);
    // Distance[3:0]
    distance_raw_one[1] = I2C1_Read1ByteRegister(0x80 >>1,0x5F);    
    return ((distance_raw_one[0] * 16 + distance_raw_one[1])/16/4);
}

void SetUpModeHalf_step()
{
    hMS1_LEFT  = HIGH;
    hMS2_LEFT  = LOW;    
    hMS1_RIGHT = HIGH;
    hMS2_RIGHT = LOW;
    hMS1_DP    = LOW;
    hMS2_DP    = HIGH;
}

void ManualHandle()
{
    char bufferManual_t[20];
    if((buffer_sys[0] == 'M') || (flagManual == 1)){
        flagManual = 1;
        for(int i = 0; i < strlen(buffer_sys); i++){
            if(buffer_sys[0] == 'M'){
                buffer_sys[0] = '0';      
            }            
            if((buffer_sys[i] == 'v') || (buffer_sys[i] == '#')){
                if((buffer_sys[i] == 'v')){
                    velManual = atoi(bufferManual_t);
                    for(int j = 0; j < strlen(bufferManual_t);j++){    
                        bufferManual_t[j] = ' ';
                    }
                    cntManual = 0;
                }                
                if(buffer_sys[i] == '#'){
                    flagManual = 0;
                    // cntCamera = 0;  
                    for(int j = 0; j < strlen(bufferManual_t);j++){                   
                        bufferManual_t[j] = ' ';
                    }
                }
            }
            else{                
                bufferManual_t[cntManual] = buffer_sys[i];
                cntManual = cntManual + 1;                
            }
        }
    }
}

void ManualControl()
{
    if(buffer_sys[0] == 'T'){ // Robot go forward
        cnt_sys = 0;
        totalTime  = 100000;
        velManual_t = velManual;
        if(velManual < 0) velManual = -velManual;
        if(in_ramp) ramp_step_function(velManual,0,100);
        velocity_sys = velManual;
    }           
    else if(buffer_sys[0] == 'B'){ // Robot go backward
        cnt_sys = 0;
        totalTime  = 100000;
        velManual_t = velManual;
        if(velManual > 0) velManual = -velManual;
        if(in_ramp) ramp_step_function(velManual,0,100);
        velocity_sys = velManual;
    } 
    else if(buffer_sys[0] == 'L'){ // Robot turn left
        cnt_sys = 0;
        totalTime    = 100000;
        if(velManual != 0) velManual_t  = velManual;
        velManual    = 0;
        omegaManual  = OMEGA_MAX/2;
        velocity_sys = velManual;
        omega_sys    = omegaManual;
    }
    else if(buffer_sys[0] == 'R'){ // Robot turn right
        cnt_sys      = 0;
        totalTime    = 100000;
        if(velManual != 0) velManual_t  = velManual;
        velManual    = 0;        
        omegaManual  = -OMEGA_MAX/2;
        velocity_sys = velManual;
        omega_sys    = omegaManual;
    }
    else if(buffer_sys[0] == 'D'){ // Camera go up
        cnt_sys = 0;
        if(t_dp == T_MAX)  t_dp = 32;
        if(hDIR_DP == LOW) hDIR_DP = HIGH;
    }
    else if(buffer_sys[0] == 'U'){ // Camera go down
        cnt_sys = 0;
        if(t_dp == T_MAX)  t_dp = 32;
        if(hDIR_DP == HIGH) hDIR_DP = LOW;
    }
    else if(buffer_sys[0] == 'N'){ // Robot stop
        cnt_sys      = 0;
        if(velManual_t != velManual) velManual = velManual_t;
        if(in_ramp == 0) ramp_step_function(0,0,100);
        omegaManual  = 0;
        cntTime      = 0;
        totalTime    = totalTimeMin;
        velocity_sys = 0;
        omega_sys    = 0;
        in_ramp      = 1;
        
        t_dp = T_MAX; //Motor of control camera direction is off
    }    
}

void AutoCircleHandle()
{
    char bufferACircle_t[20];
    if((buffer_sys[0] == 'A') || (flagACircle == 1)){
        flagACircle = 1;
        for(int i = 0; i < strlen(buffer_sys); i++){
            if(buffer_sys[0] == 'A'){
                buffer_sys[0] = '0';      
            }            
            if((buffer_sys[i] == 'v') || (buffer_sys[i] == 'r') || (buffer_sys[i] == 'a') || (buffer_sys[i] == '#')){
                if((buffer_sys[i] == 'v')){
                    velACircle = atoi(bufferACircle_t);
                    //printf("velACircle: %f\n",velACircle);
                    for(int j = 0; j < strlen(bufferACircle_t);j++){    
                        bufferACircle_t[j] = ' ';
                    }
                    cntACircle = 0;
                }
                if(buffer_sys[i] == 'r'){
                    valRadius_AC = atoi(bufferACircle_t);
                    //printf("valRadius_AC: %f\n",valRadius_AC);
                    for(int j = 0; j < strlen(bufferACircle_t);j++){                    
                        bufferACircle_t[j] = ' ';
                    }
                    cntACircle = 0;      
                }        
                if(buffer_sys[i] == 'a'){
                    angle_ACircle = atoi(bufferACircle_t);
                    //printf("angle_ACircle: %d\n",angle_ACircle);
                    for(int j = 0; j < strlen(bufferACircle_t);j++){                    
                        bufferACircle_t[j] = ' ';
                    }
                    cntACircle = 0;  
                }
                if(buffer_sys[i] == '#'){
                    flagACircle = 2;
                    // cntCamera = 0;  
                    for(int j = 0; j < strlen(bufferACircle_t);j++){                   
                        bufferACircle_t[j] = ' ';
                    }
                }
            }
            else{                
                bufferACircle_t[cntACircle] = buffer_sys[i];
                cntACircle = cntACircle + 1;                
            }
        }
    }    
}

void AutoCircleControl()
{
    if(flagACircle == 2){
        timeACircle   = (PI*valRadius_AC*angle_ACircle*100)/(18*velACircle); //ms
        omegaACircle  = velACircle/valRadius_AC;
        if(totalTime != timeACircle) totalTime = timeACircle; 
        cntTime       = 0;
        ramp_step_function(velACircle,valRadius_AC,timeACircle);
        velocity_sys  = velACircle;
        omega_sys     = omegaACircle;
        flagACircle   = 0;
    }
}

void AutoLineHandle()
{
    char bufferALine_t[20];
    if((buffer_sys[0] == 'S') || (flagALine == 1)){
        flagALine = 1;
        for(int i = 0; i < strlen(buffer_sys); i++){
            if(buffer_sys[0] == 'S'){
                buffer_sys[0] = '0';      
            }            
            if((buffer_sys[i] == 'v') || (buffer_sys[i] == 'l') || (buffer_sys[i] == 'd') || (buffer_sys[i] == '#')){
                if((buffer_sys[i] == 'v')){
                    velALine = atoi(bufferALine_t);
                    //printf("velALine: %f\n",velALine);
                    for(int j = 0; j < strlen(bufferALine_t);j++){    
                        bufferALine_t[j] = ' ';
                    }
                    cntALine = 0;
                }
                if(buffer_sys[i] == 'l'){
                    length_ALine = atoi(bufferALine_t);
                    //printf("length_ALine: %d\n",length_ALine);
                    for(int j = 0; j < strlen(bufferALine_t);j++){                    
                        bufferALine_t[j] = ' ';
                    }
                    cntALine = 0;      
                }        
                if(buffer_sys[i] == 'd'){
                    direct_ALine = atoi(bufferALine_t);                    
                    //printf("direct_ALine: %d\n",direct_ALine);
                    for(int j = 0; j < strlen(bufferALine_t);j++){                    
                        bufferALine_t[j] = ' ';
                    }
                    cntALine = 0;  
                }
                if(buffer_sys[i] == '#'){
                    if(direct_ALine != 0) flagALine = 2; 
                    else flagALine = 3;
                    // cntCamera = 0;  
                    for(int j = 0; j < strlen(bufferALine_t);j++){                   
                        bufferALine_t[j] = ' ';
                    }
                }
            }
            else{                
                bufferALine_t[cntALine] = buffer_sys[i];
                cntALine = cntALine + 1;                
            }
        }
    }
}

void AutoLineControl()
{
    if(flagALine == 2){
        if(direct_ALine < 0){
            direct_ALine = -direct_ALine;
            omegaALine = -3;
        }
        else omegaALine = 3;
        timeALine = (2*PI*btw_2wheels*direct_ALine*100)/3753; //ms //w = 3  r = 69.5
        cntTime = 0; 
        if(totalTime != timeALine) totalTime = timeALine-135;                       
        KE(omegaALine,0);        
        while(cntTime < totalTime);
        flagALine = 3;
        timeALine = 0;        
    }
    if (flagALine == 3){
        timeALine     = (length_ALine/velALine)*1000;
        if(totalTime != timeALine) totalTime = timeALine;
        cntTime       = 0;
        omegaALine    = 0;
        velocity_sys  = velALine;
        omega_sys     = omegaALine;
        flagALine     = 0;
    }
}

void TimeLapseHandle()
{
    char bufferCamera_t[20];
    if((buffer_sys[0] == 'C') || (flagCamera == 1)){
        flagCamera = 1;
        for(int i = 0; i < strlen(buffer_sys); i++){
            if(buffer_sys[0] == 'C'){
                buffer_sys[0] = '0';      
            }            
            if((buffer_sys[i] == 'r') || (buffer_sys[i] == 'l') || (buffer_sys[i] == 't') || (buffer_sys[i] == '#')){
                if((buffer_sys[i] == 'r')){
                    valRadius_TL = atoi(bufferCamera_t);
                    //printf("valRadius_TL: %f\n",valRadius_TL);
                    for(int j = 0; j < strlen(bufferCamera_t);j++){    
                        bufferCamera_t[j] = ' ';
                    }
                    cntCamera = 0;
                }
                if(buffer_sys[i] == 'l'){
                    valVLength_TL = atoi(bufferCamera_t);
                    //printf("valVLength_TL: %f\n",valVLength_TL);
                    for(int j = 0; j < strlen(bufferCamera_t);j++){                    
                        bufferCamera_t[j] = ' ';
                    }
                    cntCamera = 0;      
                }        
                if(buffer_sys[i] == 't'){
                    valTime_TL = atoi(bufferCamera_t);
                    //printf("valTime_TL: %f\n",valTime_TL);
                    for(int j = 0; j < strlen(bufferCamera_t);j++){                    
                        bufferCamera_t[j] = ' ';
                    }
                    cntCamera = 0;  
                }
                if(buffer_sys[i] == '#'){
                    flagCamera = 2;
                    // cntCamera = 0;  
                    for(int j = 0; j < strlen(bufferCamera_t);j++){                   
                        bufferCamera_t[j] = ' ';
                    }
                }
            }
            else{                
                bufferCamera_t[cntCamera] = buffer_sys[i];
                cntCamera = cntCamera + 1;                
            }
        }
    }
    if(flagCamera == 2){
        sum_frame   = valVLength_TL*30; //fps = 30
        angle_move  = (float)360/sum_frame;
        delay_frame = (angle_move*valTime_TL*1000)/6; // angle*valTime*1000*60/360 (unit: milisecond)
        time_move   = (PI*valRadius_TL*angle_move*100)/(18*velocity_TL); //PI*R*angle*1000/180*velocity  (unit: milisecond)
        omega_TL    = velocity_TL/valRadius_TL;
        flagCamera  = 3;
    }
}

void TimeLapseControl()
{
    if(flagCamera == 3){
        if(cnt_frame <= sum_frame){
            if(flag_stop == ROBOT_STOP){
                cnt_frame++;
                EUSART1_Write('1');
                for(int i = 0; i < delay_frame; i++) __delay_ms(1);
                if(totalTime != time_move) totalTime = time_move; 
                flag_stop    = ROBOT_RUN;
                cntTime      = 0;
                velocity_sys = velocity_TL;
                omega_sys    = omega_TL;
            }
        }
        else{
            EUSART1_Write('d'); // Completed with sum_frame
            flagCamera  = 0;
        }
    }     
}

void PID()
{
    DistanceValueTop    = distance_sensor_top();     
    DistanceValueBottom = distance_sensor_bottom();
//    printf("DistanceValueTop %f \n", DistanceValueTop);
//    printf("DistanceValueBottom %f \n", DistanceValueBottom);
    errorOmega = (-DistanceValueTop + DistanceValueBottom);
    printf("%f \n", errorOmega);
    if (errorOmega_sum < 1000) errorOmega_sum += errorOmega;
    PID_Omega = Kp_Omega*errorOmega + Ki_Omega*errorOmega_sum + Kd_Omega*(errorOmega - errorOmega_pre);
    errorOmega_pre = errorOmega;                 
    if(PID_Omega > PID_OMEGA_MAX) PID_Omega = PID_OMEGA_MAX;
    if(PID_Omega < PID_OMEGA_MIN) PID_Omega = PID_OMEGA_MIN;
    totalTime    = 100000;
    cntTime = 0;
    omega_sys    = PID_Omega;
    velocity_sys = 0;
}

//void UART1_isr(){
//  //  if(EUSART1_is_rx_ready()){          
//           buffer_sys[i] = EUSART1_Read();
//   //        PIR1bits.RC1IF=0;
//           EUSART2_Write(buffer_sys[i]);
//           i++;
//  //      }
//}

void main(void)
{
    // initialize the device   
    SYSTEM_Initialize();
    TMR1_SetInterruptHandler(T1_ROBOT_ISR);
    //  EUSART1_SetRxInterruptHandler(UART1_isr);

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
      INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
      INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    // INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    LATCbits.LATC2 = HIGH;
    SetUpModeHalf_step();
    while (1){
//        EUSART1_Write('1');
//        __delay_ms(3000);
        if(EUSART1_is_rx_ready()){          
           buffer_sys[cnt_sys] = EUSART1_Read();
           EUSART2_Write(buffer_sys[cnt_sys]);
           cnt_sys = cnt_sys + 1;
        }
        if(buffer_sys[cnt_sys - 1] == '#') {
            ManualHandle();
            AutoCircleHandle();
            AutoLineHandle();
            TimeLapseHandle();
            cnt_sys = 0;
        }
        ManualControl();
        AutoCircleControl();
        AutoLineControl();
        TimeLapseControl();
          //PID();
        KE(omega_sys, velocity_sys);
////        printf("omega_sys %f \n", omega_sys);
////        printf("velocity_sys %f \n", velocity_sys);
    }
}
