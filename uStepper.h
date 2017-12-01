/*
      uStepper -- Simple Stepper Driver Library
      Copyright (C) 2016  Phillip J Schmidt
      
         This program is free software: you can redistribute it and/or modify
         it under the terms of the GNU General Public License as published by
         the Free Software Foundation, either version 3 of the License, or
         (at your option) any later version.
         
         This program is distributed in the hope that it will be useful,
         but WITHOUT ANY WARRANTY; without even the implied warranty of
         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
         GNU General Public License for more details.
         
         You should have received a copy of the GNU General Public License
         along with this program.  If not, see <http://www.gnu.org/licenses/>

 */
 
#ifndef uStepper_h
   #define uStepper_h
   
   #include <arduino.h>

   class uStepper
   {
      public:
      
         uStepper(float _stepsPerMM, int direction, float _tickRateHz, int _stepPin, int _dirPin, int _enablePin);
         ~uStepper();
         
         void setSpeed(int feedRate);
         void setTickRateHz(const uint32_t &  _tickRateHz);
         void setPositionMM(const float & posFloat);
         void setPosition(const int32_t & posInt);
         
         float getPositionMM();
         int32_t getPositionSteps();
         float getSpeed();

         void enable();
         void disable();
         
         inline void step();  // call from ISR


      private:
      
         void stepPulseOff();
         void stepPulseOn();
         void setMinVelocity(float minVel);
         
         boolean stepPinOn;
         boolean enabled;
         
         float stepsPerMM, MMPerStep;
         int directionPin, stepPin, enablePin;
         int FORWARD, REVERSE;
         
         volatile int ditherTotalSteps, ditherLongSteps, ditherCounter;

         int maxFeedRate, minFeedRate;
         
         volatile int32_t  position;
         
         volatile uint32_t  tickPerStep;

         unsigned int tickCounter;
         float tickPerMin;
         
         float velocity;
         
         enum moveDir_t {
            Positive,
            Negative,
            Stopped
         } moveDirection;

   };
   
   
   // defined in header to allow "inline" declaration
   inline void uStepper::step(){  // call from ISR
      
      // This is kept fast by only executing an increment and a comparison on most calls (depending on speed)
      tickCounter++;

      if(tickCounter >= tickPerStep) // send step to motor
      { 
         // This is executed twice:
         //    * first to set the pulse pin high
         //    * second to set it low (on the following call)
         //
         // This insures sufficient time for the stepper controller to see the pulse, without adding an artificial delay into the ISR
      
         if(stepPinOn)    // check if pin is already high 
         { 
            stepPulseOff();
         }
         else
         {
            stepPulseOn();
         }
      }
   }

   
#endif