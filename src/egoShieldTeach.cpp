/********************************************************************************************
*       File:       egoShieldTeach.cpp                                                      *
*       Version:    1.1.0                                                                   *
*       Date:       March 17th, 2018                                                        *
*       Author:     Mogens Groth Nicolaisen                                                 *
*                                                                                           * 
*********************************************************************************************
*                 egoShield class                                                           *
*                                                                                           *
* This file contains the implementation of the class methods, incorporated in the           *
* egoShield Arduino library. The library is used by instantiating an egoShield object       *
* by calling of the overloaded constructor:                                                 *
*                                                                                           *
*   example:                                                                                *
*                                                                                           *
*   egoShield ego;                                                                          *
*                                                                                           *
* The instantiation above creates an egoShield object                                       *
* after instantiation of the object, the object setup function should be called within      *
* Arduino's setup function, and the object loop function should be run within the Arduino's *
* loop function:                                                                            *
*                                                                                           *
*   example:                                                                                *
*                                                                                           *
*   egoShieldTeach ego;                                                                     *
*                                                                                           *
*   void setup()                                                                            *
*   {                                                                                       *
*     ego.setup();                                                                          *
*   }                                                                                       *
*                                                                                           *
*   void loop()                                                                             *
*   {                                                                                       *
*     ego.loop();                                                                           *
*   }                                                                                       *
*                                                                                           *
*                                                                                           *
*********************************************************************************************
* (C) 2018                                                                                  *
*                                                                                           *
* uStepper ApS                                                                              *
* www.ustepper.com                                                                          *
* administration@ustepper.com                                                               *
*                                                                                           *
* The code contained in this file is released under the following open source license:      *
*                                                                                           *
*     Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International               *
*                                                                                           *
*   The code in this file is provided without warranty of any kind - use at own risk!       *
*   neither uStepper ApS nor the author, can be held responsible for any damage             *
*   caused by the use of the code contained in this file !                                  *
*                                                                                           *
********************************************************************************************/
/**
 * @file egoShieldTeach.cpp
 * @brief      Class implementations for the egoShield Teach library
 *
 *             This file contains the implementations of the classes defined in
 *             egoShieldTeach.h
 *
 * @author     Mogens Groth Nicolaisen (mogens@ustepper.com)
 */
 
#include "egoShieldTeach.h"

egoShield *egoPointer;

extern "C" {
  void WDT_vect(void)
  {
    egoPointer->inputs();     //examine buttons
    WDTCSR |= (1<<WDIE);      //Enable Watchdog interrupt
  }
}

egoShield::egoShield(void)
{
  u8g2 = new U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI(U8G2_R0, /* clock=*/ 11, /* data=*/ 9, /* cs=*/ U8X8_PIN_NONE, /* dc=*/ 2, /* reset=*/ 10);
}

void egoShield::setup(uint16_t acc, uint16_t vel, uint8_t uStep, uint16_t fTol, uint16_t fHys, float P, float I, float D, float res)//brake mode?
{
  egoPointer = this;    //We need a global pointer with the address of the egoShield object, for the watchdog interrupt handler to access the object

  cli();                //Make sure we dont get interrupted while initializing
  //Setup Watchdog timer, no prescaler (16ms interrupt period)
  RESETWDT;           
  WDTCSR = (1 << WDCE) | (1 << WDE);
  WDTCSR |= (1 << WDIE) | (1 << WDE);

  //Store initialization inputs in egoShield object
  this->acceleration = acc;
  this->velocity = vel;
  this->microStepping = uStep;
  this->faultTolerance = fTol;
  this->faultHysteresis = fHys;
  this->pTerm = P;
  this->iTerm = I;
  this->dTerm = D;
  this->resolution = res;
  this->stepSize = 2;
  this->interval = 2000;

  brakeFlag = 1;    //Flag to indicate whether brake should be activated or not

  stepper.setup(PID,this->microStepping,this->faultTolerance,this->faultHysteresis,this->pTerm,this->iTerm,this->dTerm,1);    //initialize ustepper object
  u8g2->begin();//start display
  
  // Check whether the uStepper is mounted on a motor with a magnet attached. If not, show an error message untill mounted correctly
  do
  {
    u8g2->firstPage();
    do 
    {
      u8g2->setFontMode(1);
      u8g2->setDrawColor(1);
      u8g2->setFontDirection(0);
      u8g2->setFont(u8g2_font_6x10_tf);
    
      u8g2->drawStr(2,10,"Magnet not present !");
    } while ( u8g2->nextPage() );  
  }
  while(stepper.encoder.detectMagnet() == 2 || stepper.encoder.detectMagnet() == 1);

  this->startPage();//show startpage
  stepper.encoder.setHome();      //Set home to current position
  stepper.setMaxVelocity(this->velocity);
  stepper.setMaxAcceleration(this->acceleration);
  stepper.moveToEnd(1);
  stepper.moveToAngle(30,HARD);
  while(stepper.getMotorState());
  stepper.encoder.setHome();
  pidFlag = 1;//enable PID
  //Setup IO pins
  pinMode(FWBT ,INPUT);
  pinMode(PLBT ,INPUT);
  pinMode(RECBT ,INPUT);
  pinMode(BWBT ,INPUT);
  pinMode(OPTO,OUTPUT);
  digitalWrite(OPTO ,HIGH);
  digitalWrite(FWBT ,HIGH);//pull-up
  digitalWrite(PLBT ,HIGH);//pull-up
  digitalWrite(RECBT ,HIGH);//pull-up
  digitalWrite(BWBT ,HIGH);//pull-up
  setPoint = stepper.encoder.getAngleMoved();//set manual move setpoint to current position
  
  
  delay(2000);//for 2 seconds
  this->resetAllButton();   //Initialize buttons 
  state = 'a';//start in idle
}

void egoShield::loop(void)
{
  setPoint = stepper.encoder.getAngleMoved();   
  switch (state)
  {
    case 'a'://if we are in idle
      idleMode();
    break;
    
    case 'b'://if we are in play
        playMode();
    break;
    
    case 'c'://if we are in record
        recordMode();
    break;
  
    case 'd'://in pause
      pauseMode();
    break;
  }
}

void egoShield::idleMode(void)
{
  static bool continousForward = 0;
  static bool continousBackwards = 0;

  this->idlePage(pidFlag,setPoint);

  if(continousForward)
  {
    if(this->forwardBtn.state != HOLD)
    {
      stepper.hardStop(HARD);
      continousForward = 0;
    }
  }
  else if(continousBackwards)
  {
    if(this->backwardsBtn.state != HOLD)
    {
      stepper.hardStop(HARD);
      continousBackwards = 0;
    }
  }
  if(this->playBtn.btn)//if play/stop/pause is pressed for long time, invert the pid mode
  {
    while(this->playBtn.state == PRESSED);
    if(this->playBtn.state == DEPRESSED)//we want to play sequence when doing a short press
    {
      state = 'b';
      continousForward = 0;
      continousBackwards = 0;
      this->resetAllButton(); 
    }
    else
    {
      if(pidFlag == 0)
      {
        pidFlag = 1;
        stepper.setup(PID,this->microStepping,this->faultTolerance,this->faultHysteresis,this->pTerm,this->iTerm,this->dTerm,0);//pause PID to allow manual movement
      }
      else
      {
        pidFlag = 0;
        stepper.setup(NORMAL,this->microStepping,this->faultTolerance,this->faultHysteresis,this->pTerm,this->iTerm,this->dTerm,0);//pause PID to allow manual movement
        stepper.hardStop(SOFT);
      }
      this->idlePage(pidFlag,setPoint);
      while(this->playBtn.state == HOLD);
    }
    this->resetButton(&playBtn);
  }
  else if(this->forwardBtn.btn)//if manual forward signal
  {
    if(this->forwardBtn.state == HOLD)
    {
      if(!continousForward)
      {
        stepper.runContinous(CCW);
        continousForward = 1;  
      }
      this->forwardBtn.btn = 0;
    }
    else
    {
      stepper.moveAngle(-5.0,0);//move 5deg
      this->forwardBtn.btn = 0;
    }
  }
  else if(this->backwardsBtn.btn)//if manual backward signal
  {
    if(this->backwardsBtn.state == HOLD)
    {
      if(!continousBackwards)
      {
        stepper.runContinous(CW);
        continousBackwards = 1;  
      }
      this->backwardsBtn.btn = 0;
    }
    else
    {
      stepper.moveAngle(5.0,0);//move 5deg
      this->backwardsBtn.btn = 0;
    }
  }
  else if(this->recordBtn.btn == 1)
  {
    this->resetAllButton(); 
    continousForward = 0;
    continousBackwards = 0;
    state = 'c';
  }
}

void egoShield::playMode(void)
{
  static uint8_t started = 0;

  this->playPage(loopMode,pidFlag,place,0);
  
  if(this->recordBtn.btn)//play/stop/pause
  {
    while(this->recordBtn.state == PRESSED);
    if(this->recordBtn.state == DEPRESSED)//we want to play sequence when doing a short press
    {
        state = 'd';
        this->resetAllButton(); 
        return;
    }
  }
  else if(this->playBtn.btn)//play/stop/pause
  {
    while(this->playBtn.state == PRESSED);
    if(this->playBtn.state == DEPRESSED)//we want to play sequence when doing a short press
    {
      started = 1;
    }
    else //Long press = stop
    {
      place = 0;//reset array counter
      loopMode = 0;
      started = 0;
      state = 'a';//idle
      this->idlePage(pidFlag,setPoint);
      while(this->playBtn.state == HOLD);
    }
    this->resetAllButton(); 
    return;
  }
  else if(started || loopMode)
  {
    if(!stepper.getMotorState())
    {
      place++;//increment array counter
      if(loopMode && place > endmove)
      {
        place = 0;
      }
      else if(place > endmove)//If we are at the end move
      {
        place = 0;//reset array counter
        started = 0;
        state = 'a';
        this->resetAllButton(); 
        return;
      }
      stepper.setMaxVelocity(this->velocity);
      stepper.setMaxAcceleration(this->acceleration);
      stepper.moveToAngle(pos[place],brakeFlag);   
    }
  }
  
  if(this->forwardBtn.state == HOLD)//loop mode start
  {
    this->resetButton(&forwardBtn);
    loopMode = 1;
  }
  if(this->forwardBtn.btn)//if manual backward signal
  {
    while(this->forwardBtn.state == PRESSED);
    if(this->forwardBtn.state == HOLD)
    {
      loopMode = 1;
      this->playPage(loopMode,pidFlag,place,0);
      while(this->forwardBtn.state == HOLD);
      this->forwardBtn.btn = 0;
    }
    else
    {
      changeVelocity(1);
      this->forwardBtn.btn = 0;
    }
  }
  else if(this->backwardsBtn.btn)//if manual backward signal
  {
    while(this->backwardsBtn.state == PRESSED);
    if(this->backwardsBtn.state == HOLD)
    {
      loopMode = 0;
      this->playPage(loopMode,pidFlag,place,0);
      while(this->backwardsBtn.state == HOLD);
      this->backwardsBtn.btn = 0;
    }
    else
    {
      changeVelocity(0);
      this->backwardsBtn.btn = 0;
    }
  }
}

void egoShield::changeVelocity(bool speedDirection)
{
  if(speedDirection && this->velocity <= 9900 && this->acceleration <= 19900)//increase speed
  {
    this->forwardBtn.btn = 0;
    this->velocity+=100;
    this->acceleration+=100;
  }
  else if(!speedDirection && this->velocity >= 200 && this->acceleration >= 200)//decrease speed
  {
    this->backwardsBtn.btn = 0;
    this->velocity-=100;
    this->acceleration-=100;
  }
}

void egoShield::recordMode(void)
{
  static bool continousForward = 0;
  static bool continousBackwards = 0;
  
  this->recordPage(pidFlag,0,place,setPoint);

  if(continousForward)
  {
    if(this->forwardBtn.state != HOLD)
    {
      stepper.hardStop(HARD);
      continousForward = 0;
    }
  }
  else if(continousBackwards)
  {
    if(this->backwardsBtn.state != HOLD)
    {
      stepper.hardStop(HARD);
      continousBackwards = 0;
    }
  }
  if(this->forwardBtn.btn)//if manual forward signal
  {
    if(this->forwardBtn.state == HOLD)
    {
      if(!continousForward)
      {
        stepper.runContinous(CCW);
        continousForward = 1;  
      }
      this->forwardBtn.btn = 0;
    }
    else
    {
      stepper.moveAngle(-5.0,0);//move 5deg
      this->forwardBtn.btn = 0;
    }
  }
  else if(this->backwardsBtn.btn)//if manual backward signal
  {
    if(this->backwardsBtn.state == HOLD)
    {
      if(!continousBackwards)
      {
        stepper.runContinous(CW);
        continousBackwards = 1;  
      }
      this->backwardsBtn.btn = 0;
    }
    else
    {
      stepper.moveAngle(5.0,0);//move 5deg
      this->backwardsBtn.btn = 0;
    }
  }
  else if(this->recordBtn.btn == 1)//record position
  {      
    this->recordBtn.btn = 0;
    if(record == 0)//If we were not recording before
    {
      //stepper.encoder.setHome();//Set current position as home
      //setPoint = 0;
      place = 0;//Reset the array counter
      record = 1;//Record flag
    }
    this->recordPage(pidFlag,1,place,setPoint);
    delay(500);
    if(record == 1)//If we have initialized recording
    {
      pos[place] = setPoint;//Save current position
      place++;//Increment array counter
      if(place>CNT)
      {
        place=0;
      }
    }
  }
  else if(this->playBtn.btn == 1)//stop pressed
  {
    endmove = place-1;//set the endmove to the current position
    place = 0;//reset array counter
    record = 0;//reset record flag
    state = 'a';//stop state
    continousForward = 0;
    continousBackwards = 0;
    this->idlePage(pidFlag,setPoint);
    while(this->playBtn.state == HOLD);
    this->resetAllButton(); 
  }  
}

void egoShield::pauseMode(void)
{
  this->pausePage(loopMode,pidFlag,place);
  if(this->playBtn.btn)//play/stop/pause
  {
    while(this->playBtn.state == PRESSED);
    if(this->playBtn.state == DEPRESSED) //Short press = unpause
    {
      state = 'b';
    }
    else      //Long press = stop
    {
      state = 'a';
      this->idlePage(pidFlag,setPoint);
      while(this->playBtn.state == HOLD);
      this->resetAllButton();
    }
    this->resetButton(&playBtn);
  }
}

void egoShield::inputs(void)
{
    this->debounce(&forwardBtn,(PINC >> 3) & 0x01);
    this->debounce(&playBtn,(PINC >> 1) & 0x01);
    this->debounce(&recordBtn,(PINC >> 2) & 0x01);
    this->debounce(&backwardsBtn,(PINC >> 0) & 0x01);
}

void egoShield::startPage(void)
{
  u8g2->firstPage();
  do {
    u8g2->drawXBM(19, 20, logo_width, logo_height, logo_bits);
  } while ( u8g2->nextPage() );
}

void egoShield::idlePage(bool pidMode, float pos)
{
  char buf[20];
  String sBuf;

  sBuf = "Position: ";
  sBuf += (int32_t)(pos/this->resolution);
  sBuf += " mm";
  sBuf.toCharArray(buf, 20);

  u8g2->firstPage();
  do {
    u8g2->drawBox(1, 1, 128, 12);
    u8g2->drawBox(1, 48, 128, 68);
    u8g2->setFontMode(0);
    u8g2->setDrawColor(0);
    u8g2->setFontDirection(0);
    u8g2->setFont(u8g2_font_6x10_tf);
    
    //Bottom bar
    u8g2->drawXBM(5, 51, en_width, en_height, bw_bits);
    u8g2->drawXBM(112, 51, en_width, en_height, fw_bits);
    u8g2->drawXBM(32, 50, play_width, play_height, play_bits);
    u8g2->drawXBM(43, 51, tt_width, tt_height, stop_bits);
    u8g2->drawXBM(71, 51, tt_width, tt_height, rec_bits);
    u8g2->drawXBM(85, 51, tt_width, tt_height, pse_bits);

    //Mode
    u8g2->drawStr(2,10,"Idle");
    if(pidMode)
    {
      u8g2->drawStr(45,10,"PID ON");
    }
    else
    {
      u8g2->drawStr(45,10,"PID OFF");
    }
    u8g2->setFontMode(1);
    u8g2->setDrawColor(1);
    u8g2->drawStr(2,35,buf);
  } while ( u8g2->nextPage() );  
}

void egoShield::recordPage(bool pidMode, bool recorded, uint8_t index, float pos)
{
  char buf[22];//char array buffer
  String sBuf;
    
  u8g2->firstPage();
  do 
  {
    u8g2->drawBox(1, 1, 128, 12);
    u8g2->drawBox(1, 48, 128, 68);
    u8g2->setFontMode(0);
    u8g2->setDrawColor(0);
    u8g2->setFontDirection(0);
    u8g2->setFont(u8g2_font_6x10_tf);
    
    u8g2->drawXBM(5, 51, en_width, en_height, bw_bits);
    u8g2->drawXBM(112, 51, en_width, en_height, fw_bits);
    u8g2->drawXBM(38, 51, tt_width, tt_height, stop_bits);
    u8g2->drawXBM(76, 51, tt_width, tt_height, rec_bits);

    //Mode
    u8g2->drawStr(2,10,"Record");
    if(pidMode)
    {
      u8g2->drawStr(45,10,"PID ON");
    }
    else
    {
      u8g2->drawStr(45,10,"PID OFF");
    }
    u8g2->setFontMode(1);
    u8g2->setDrawColor(1);
    if(recorded)
    {
      sBuf = "Position ";
      sBuf += index;
      sBuf += " recorded";
      sBuf.toCharArray(buf, 22);
      u8g2->drawStr(2,35,buf);
    }
    else
    {
    sBuf = "Position: ";
    sBuf += (int32_t)(pos/this->resolution);
    sBuf += " mm";
    sBuf.toCharArray(buf, 22);
    u8g2->drawStr(2,35,buf);
    }
  } while ( u8g2->nextPage() );  
}

void egoShield::playPage(bool loopMode, bool pidMode, uint8_t index, bool mode)
{
  char buf[5];//char array buffer
  
  u8g2->firstPage();
  do 
  {
    u8g2->drawBox(1, 1, 128, 12);
    u8g2->drawBox(1, 48, 128, 68);
    u8g2->setFontMode(0);
    u8g2->setDrawColor(0);
    u8g2->setFontDirection(0);
    u8g2->setFont(u8g2_font_6x10_tf);

    if(loopMode)
    {
      u8g2->drawXBM(110, 2, loop_width, loop_height, loop_bits);
    }
    
    //Bottom bar
    u8g2->drawXBM(5, 51, en_width, en_height, bw_bits);
    u8g2->drawXBM(112, 51, en_width, en_height, fw_bits);
    u8g2->drawXBM(32, 50, play_width, play_height, play_bits);
    u8g2->drawXBM(43, 51, tt_width, tt_height, stop_bits);
    u8g2->drawXBM(77, 51, tt_width, tt_height, pse_bits);

    //Mode
    u8g2->drawStr(2,10,"Play");
    if(pidMode)
    {
      u8g2->drawStr(45,10,"PID ON");
    }
    else
    {
      u8g2->drawStr(45,10,"PID OFF");
    }
    u8g2->setFontMode(1);
    u8g2->setDrawColor(1);
    if(mode)
    {
      //u8g2->drawStr(2,25,"Adjust velocity");
    }
    else
    {
      u8g2->drawStr(2,25,"Moving to pos");
      String(index).toCharArray(buf, 5);
      u8g2->drawStr(90,25,buf);
    }
    u8g2->drawStr(2,40,"Speed:");
    String(this->velocity).toCharArray(buf, 5);
    u8g2->drawStr(60,40,buf);
  } while ( u8g2->nextPage() );
}

void egoShield::pausePage(bool loopMode, bool pidMode, uint8_t index)
{
  char buf[3];//char array buffer
    
  u8g2->firstPage();
  do 
  {
    u8g2->drawBox(1, 1, 128, 12);
    u8g2->drawBox(1, 48, 128, 68);
    u8g2->setFontMode(0);
    u8g2->setDrawColor(0);
    u8g2->setFontDirection(0);
    u8g2->setFont(u8g2_font_6x10_tf);
    
    if(loopMode)
    {
      u8g2->drawXBM(110, 2, loop_width, loop_height, loop_bits);
    }
    
    //Bottom bar
    u8g2->drawXBM(32, 50, play_width, play_height, play_bits);
    u8g2->drawXBM(43, 51, tt_width, tt_height, stop_bits);

    //Mode
    u8g2->drawStr(2,10,"Pause");
    if(pidMode)
    {
      u8g2->drawStr(45,10,"PID ON");
    }
    else
    {
      u8g2->drawStr(45,10,"PID OFF");
    }
    u8g2->setFontMode(1);
    u8g2->setDrawColor(1);
    u8g2->drawStr(2,35,"Paused at pos");
    String(index).toCharArray(buf, 3);
    u8g2->drawStr(90,35,buf);
  } while ( u8g2->nextPage() );  
}

//BT skal ikke have helt samme funktionalitet som oled. Hvis man vil køre frem eller tilbage manuelt, så skal man skrive x antal grader plus eller minus. Man skal også kunne stoppe
//mens den kører... Skærm skal vise simpel menu, hvor man kan se status på pid on/off og hvad mode man er i. Mulighederne skal også fremgå af skærmen, e.g. p = play
//encoder pos skal også opdateres vi bt. Vi skal kunne cleare terminalen for at få en ny menu?

void egoShield::debounce(buttons *btn, uint8_t sample)
{
  if(btn->state == DEPRESSED)
    {
    btn->debounce &= (0xFE + sample);

        if( (btn->debounce & 0x1F) == 0x00)
        {
            btn->state = PRESSED;
            btn->btn = 1;
            return;
        }

        btn->debounce <<= 1;
        btn->debounce |= 0x01;
    }

    else if((btn->state == PRESSED) || (btn->state == HOLD))
    {
        btn->debounce |= sample;

        if(btn->state != HOLD)
        {
          if((btn->debounce & 0x1F) == 0x00)
          {
            if(btn->holdCnt >= HOLDTIME)
            {
              btn->state = HOLD;
            }
            btn->holdCnt++;
          }
        }

        if( (btn->debounce & 0x1F) == 0x1F)
        {
            btn->state = DEPRESSED;
            btn->holdCnt = 0;
            return;
        }

        btn->debounce <<= 1;
        btn->debounce &= 0xFE;
    }

    if(btn->state == HOLD)
    {
      if(btn->time == HOLDTICK)
      {
        btn->btn = 1;
        btn->time = 0;
      }

      else
      {
        btn->time++;
      }
    }
}

void egoShield::resetButton(buttons *btn)
{
  btn->time = 0;
  btn->state = DEPRESSED;
  btn->debounce = 0x1F;
  btn->holdCnt = 0;
  btn->btn = 0;
}
void egoShield::resetAllButton()
{
    this->resetButton(&playBtn);
    this->resetButton(&forwardBtn);
    this->resetButton(&backwardsBtn);
    this->resetButton(&recordBtn);
}
