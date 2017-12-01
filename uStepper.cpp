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

#include "uStepper.h"


// ***************************
//     PUBLIC FUNCTIONS
// ***************************


uStepper::uStepper(float _stepsPerMM, int direction, float _tickRateHz, int _stepPin, int _dirPin, int _enablePin){
   
   MMPerStep  = 1.0f / _stepsPerMM; // [mm/step]
   stepsPerMM = _stepsPerMM;        // [steps/mm]
   
   setTickRateHz(_tickRateHz);
   
   position = 0;
   tickPerStep = 65535;
   moveDirection = Stopped;
   ditherCounter = 1;
   
   // Config Step Pin
   stepPin = _stepPin;
   pinMode(stepPin, OUTPUT);
   digitalWrite(stepPin, LOW);
   stepPinOn = false;
   
   // Config Direction Pin
   directionPin = _dirPin;
   pinMode(directionPin, OUTPUT);
   digitalWrite(directionPin, LOW);
   
   if(direction > 0){
      FORWARD = 1;
      REVERSE = 0;
   }else{
      FORWARD = 0;
      REVERSE = 1;      
   }

   // Config Enable Pin
   enablePin = _enablePin;
   pinMode(enablePin, OUTPUT);
   disable();

}


uStepper::~uStepper()
{
}


void uStepper::setSpeed(int feedRate)    // pass in speed [MM/min]
{
   
   if( enabled )
   {
      feedRate = constrain(feedRate, -maxFeedRate, maxFeedRate);
   }
   else
   {
      feedRate = 0.0f;
   }
   
   if(feedRate < -minFeedRate) // avoid silly low speeds to prevent 16bit int overflow
   {
      digitalWrite(directionPin, REVERSE);
      moveDirection = Negative;
      velocity = float( feedRate ); // mm/s
   }
   else if(feedRate > minFeedRate)
   {
      digitalWrite(directionPin, FORWARD);
      moveDirection = Positive;
      velocity = float( feedRate ); // mm/s
   }
   else
   {
      moveDirection = Stopped;
      tickPerStep = 65535;
      velocity = 0.0f; // mm/s
      return;  // exit now
   }
   
   float feedRateABS = float(abs(feedRate));
   
   float stepsPerMin = feedRateABS * stepsPerMM;  // [MM/min] * [step/MM] = [steps/min]

   float idealTickPerStep = tickPerMin / stepsPerMin;  // [tick/min] / [step/min] = [tick/step]

   tickPerStep = long(idealTickPerStep); 
   
   // compute step dithering
   float offset = idealTickPerStep - float(tickPerStep); // should be a decimal between 0 and 1
   
   if (offset < 0.4999695f)
   {
      offset += 0.0000306f; // avoid divide by zero (and overflow of 16bit int)
      int temp = int( 1.0f / offset + 0.5f ); // add 0.5 to force correct up/down rounding
      
      noInterrupts(); // these variables are shared with the interupt function
      ditherTotalSteps = temp;
      ditherLongSteps  = 1; // only one long step
      interrupts();
   }
   else
   {
      offset = 1.0000306f - offset; 
      int temp = int( 1.0f / offset + 0.5f );
      
      noInterrupts(); // these variables are shared with the interupt function
      ditherTotalSteps = temp;
      ditherLongSteps  = ditherTotalSteps - 1;  // only one short step
      interrupts();
   }
   
   // *** Debug Output ***
   //Serial.print(idealTickPerStep, 2); Serial.print("\t");
   //Serial.print(int(idealTickPerStep + 0.5f)); Serial.print("\t");
   //Serial.print(tickPerStep); Serial.print("\t");
   //Serial.print(ditherTotalSteps - ditherLongSteps); Serial.print("\t");  // small steps
   //Serial.print(ditherLongSteps); Serial.print("\t");                     // large steps
   //Serial.print(float(ditherLongSteps * (tickPerStep + 1) + (ditherTotalSteps - ditherLongSteps) * tickPerStep) / float(ditherTotalSteps), 2); Serial.print("\t"); // actual interpolated speed
   //Serial.print(maxFeedRate/60); Serial.print("\t");
   //Serial.print(minFeedRate); Serial.print("\t");

}


void uStepper::setPositionMM(const float & posFloat)
{
   uint32_t posInt = posFloat * stepsPerMM + 0.5f;
   
   noInterrupts();
   position = posInt;
   interrupts();
}


void uStepper::setPosition(const int32_t & posInt)
{
   noInterrupts();
   position = posInt;
   interrupts();
}


void uStepper::setTickRateHz(const uint32_t & _tickRateHz)
{
   tickPerMin = float(_tickRateHz) * 60.0f;
   maxFeedRate = int(constrain((tickPerMin * MMPerStep) * 0.333f, 0.0f, 32767.0f)); // limit to one step every three tick or 16bit int max (~546mm/s @ 80step/mm)
   setMinVelocity(minFeedRate); // insure changes in tick rate do not result in excessively low min vel limits
}


float uStepper::getPositionMM()
{
   noInterrupts();
   int32_t temp = position;
   interrupts();
   
   return float(temp) * MMPerStep;
}


int32_t uStepper::getPositionSteps()
{
   noInterrupts();
   int32_t temp = position;
   interrupts();
   
   return temp;
}


float uStepper::getSpeed() // return velocity in mm/s
{
   return velocity * 0.01666666667f;
}


void uStepper::enable()
{
   digitalWrite(enablePin, LOW);
   moveDirection = Stopped;
   tickPerStep = 65535;
   enabled = true;
}


void uStepper::disable()
{
   digitalWrite(enablePin, HIGH);
   moveDirection = Stopped;
   tickPerStep = 65535;
   enabled = false;
}


// ***************************
//     PRIVATE FUNCTIONS
// ***************************


void uStepper::stepPulseOff()
{
   digitalWrite(stepPin, LOW);   // set pin low
   stepPinOn = false;

   // Dither control: interpolate speed by alternating between faster and slower "real" speeds
   if(ditherCounter > ditherLongSteps)
   {
      tickCounter = 1; // faster short step time 
      //Serial.print("[S] ");
      if(ditherCounter >= ditherTotalSteps) ditherCounter = 0;
   }
   else
   {
      tickCounter = 0; // slower long step time
      //Serial.print("[-L-] ");
   }
   
   ditherCounter++;   
}


void uStepper::stepPulseOn()
{
   if(moveDirection == Positive)
   {
      position++;
      digitalWrite(stepPin, HIGH);
      stepPinOn = true;
   }
   else if(moveDirection == Negative)
   {
      position--;
      digitalWrite(stepPin, HIGH);
      stepPinOn = true;
   }
   else
   {
      tickCounter = 0; // reset to avoid running repeatedly
      // no step pulse is sent if stopped
   }   
}


void uStepper::setMinVelocity(float minVel)
{
   minFeedRate = max( int(minVel + 0.5f), int(tickPerMin / (65537.0f * stepsPerMM)) ); // prevent 16bit int overflow on very low feed rates
}








