#ifndef _COMMAND_H
#define _COMMAND_H


enum command : byte { C_NONE = 0,       // a "nop" command - used for heartbeat
// Joystick forward button
                      C_GUN_OFF,        // jN click: turn gun off, rotate into center yaw, tilt fwd down, stop tracking
//					  C_BLADES,			// jN doublecick: sound of blades drawn out and back in
					  C_BLADES_OUT,		// jN lp start: sound of blades drawn out
					  C_BLADES_IN,		// jN lp stop: sound of blades drawn back in
					  C_TRACK_IMU,		// jN click/upsidedown: switch to IMU tracking mode
					  
// Joystick backwards button
                      C_GUN_RECAL,      // jS click: turn gun on (if was off), recenter and sync with head IMU (head should be positioned fwd centered).
                      // if gun was already on, just recenter and sync with head IMU

                      C_GUN_RECAL_HLD,  // jS long press: recenter the gun and hold it in that position.
                      // On release, send C_GUN_RECAL again to start tracking
					  C_TRACK_MARG,		// jS click/upsidedown: switch to MARG tracking mode
					  C_TRACK_CALIB,	// jS lp start/upsidedown: initiate MARG calibration sequence
					  
// Joystick right button
                      C_SND_CLICK,      // jE click: start playing random clicking sound
                      C_SND_ANYTIME,     // jE dbclick: play "anytime"
					  C_SND_OVERHERE,    // jE longpress: play "over here"
                      
					  C_VOL_UP,         // jE click/upsidedown: volume up

// Joystick left button
                      C_SND_ROAR,       // jW click: start playing random roar
					  C_SND_GROAL,      // jW longpress: play groaling sound
                      C_SND_LAUGH,      // jW dbclick: play laugh
					  

                      C_VOL_DOWN,       // jW click/upsidedown: volume down

// Controller Z (Fire) button

                      C_GUN_SHOT,       // zB click: one shot from the gun
                      C_GUN_BURST,      // zB long press: a burst of gunfire until the button is released
                      // On release: stop the burst (next command)

                      C_GUN_STOP,       // zB lp release: stop the gunfire burst

					  C_SYS_TEST, 		// zB long press/upsidedown: systems test routine

// Controller C button					  
					  
                      C_GUN_LASER,      // zC click: toggle laser pointer of the gun. one click on, one click off
                      C_GUN_TRACK,      // zC long press: turn laser poainter on and track that point with the gun.
                      // On release do nothing.
					  C_LED_FLASH,		// zC double click: flash yellow LEDs on the head

					  C_GUMMY,			// zC long press/upsidedown: play "i am a gummy bear"
					  C_THEME			// zC click/upsidedown" play the "predator theme" music

                    };
					
#endif  // _COMMAND_H