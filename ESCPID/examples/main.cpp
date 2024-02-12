
#include <Arduino.h>
#include "ESCPID/DSHOT.h"
#include "ESCPID/ESCCMD.h"

#define ESCPID_NB_ESC             2                 // Number of ESCs
#define ESCPID_MAX_ESC            6                 // Max number of ESCs

#define ESCPID_COMM_WD_LEVEL      20                // Maximum number of periods without reference refresh

// Globals
uint16_t  ESCPID_comm_wd = 0;
int16_t   ESC_Throttle[ESCPID_NB_ESC] = {};     // Desired throttle for motors -999 to 999

void setup() {
  int i;

  // Initialize USB serial link for DEBUG
  Serial.begin( 115200 );

  // Initialize the CMD subsystem
  ESCCMD_init( ESCPID_NB_ESC ); //disabled telemetry serial

  // Arming ESCs
  ESCCMD_arm_all( );
  
  // Switch 3D mode on
  ESCCMD_3D_on( );

  // Arming ESCs
  ESCCMD_arm_all( );
  
  // Start periodic loop
  ESCCMD_start_timer( );

  // Stop all motors
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCCMD_stop( i );
  }
  //}

  // Reference watchdog is initially triggered
  ESCPID_comm_wd = ESCPID_COMM_WD_LEVEL;

  //Analog Input Poti
  pinMode(A0, INPUT);
}

void loop() {
  static int    i, ret;

  //Serial.println(analogRead(A0));
  int16_t myRead = map(analogRead(A0), 400, 1000, 0, 100);
  if (myRead<0) myRead=0;
  ESC_Throttle[0] = myRead;
  ESC_Throttle[1] = myRead;


  if(true) { //your condition to keep the motors running safely
    ESCPID_comm_wd = 0;
  }

  // Check for next timer event
  ret = ESCCMD_tic( );  //disabled telemetry serial

  // Process timer event
  if ( ret == ESCCMD_TIC_OCCURED )  {

    // Read all measurements and compute current control signal
    for ( i = 0; i < ESCPID_NB_ESC; i++ ) {

      if ( ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL ) {
        ret = ESCCMD_throttle( i, ESC_Throttle[i] );
        Serial.print("Throttling ");
        Serial.print(i);
        Serial.print(" at ");
        Serial.println(ESC_Throttle[i]);
      }
    }
    
    // Update watchdog
    if ( ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL )  {
      ESCPID_comm_wd++;
    }
  } 
}