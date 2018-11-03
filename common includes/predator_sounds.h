#ifndef 	_PREDATOR_SOUNDS_H
#define 	_PREDATOR_SOUNDS_H

const byte 	S_FLD_SYSTEM		=	1	; //	- system
const byte 	S_MAC_BOOT			=	1	; //	 - mac boot sound // first sound of the system
const byte 	S_HALO_BOOT			=	2	; //	 - halo
const byte 	S_STARTING_UP		=	3	; //	 - starting up   // begining of boot sequence
const byte 	S_ALL_SET			=	4	; //	 - all set  // end of boot sequesnce

const byte	S_FLD_CLICKING		=	2	; //	- clicking noises // random: C_SND_CLICK,      // jE click: start playing random clicking sound
const byte 	S_CLICKING_START	=	1	; //	
const byte 	S_CLICKING_END		=	7	; //	

const byte 	S_FLD_ROARS			=	3	; //	- Roars
const byte 	S_ROARS_START		=	1	; //	 - healing  // random: C_SND_ROAR,       // jW click: start playing random roar
//const byte 		=	2	; //	 - trophy  // random: C_SND_ROAR,       // jW click: start playing random roar
const byte 	S_ROARS_END			=	3	; //	 - roar   // random: C_SND_ROAR,       // jW click: start playing random roar
const byte 	S_ROARS_SNARL		=	4	; //	 - snarl   // C_SND_GROAL,      // jW longpress: play graoling sound

const byte 	S_FLD_EQUIPMENT		=	4	; //	- equipment
const byte 	S_GUN_ACTIVE_START	=	1	; //	 - gun activation   // random: C_GUN_RECAL,      // jS click: turn gun on
//const byte 		=	2	; //	 - another gun activaton  // random: C_GUN_RECAL,      // jS click: turn gun on//
const byte 	S_GUN_ACTIVE_END	=	3	; //	 - another gun activaton  // random: C_GUN_RECAL,      // jS click: turn gun on
const byte 	S_AIM_CLICK			=	103	; //	 - aim with a click   // random with follow up: C_GUN_LASER,      // zC click: toggle laser pobyteer of the gun. one click on, one click off
const byte 	S_AIM_CLICK2		=	104	; //	 - continous aiming sound
const byte 	S_AIM_NO_CLICK		=	105	; //	 - aim w/o a click   // random with follow up: C_GUN_LASER,      // zC click: toggle laser pobyteer of the gun. one click on, one click off
const byte 	S_AIM_NO_CLICK2		=	106	; //	 - continous aiming sound 
const byte 	S_AIM_SOFT			=	107	; //	 - soft aiming sound   // random with follow up: C_GUN_LASER,      // zC click: toggle laser pobyteer of the gun. one click on, one click off
const byte 	S_AIM_SOFT2			=	108	; //	 - continous aiming sound
const byte 	S_AIM_LONG			=	120	; //	 - long aim     C_GUN_TRACK,      // zC long press: turn laser poabyteer on and track that pobyte with the gun.
const byte 	S_GUN_SHOT			=	200	; //	 - gun shot with a long charging sound // C_GUN_SHOT,       // zB click: one shot from the gun
const byte 	S_GUN_BURST			=	201	; //	 - gun burst shot with a short charging sound // C_GUN_BURST,       // zB long press: one shot from the gun
const byte 	S_BLADES_OUT		=	210	; //	 - the sound of blades drawn out
const byte 	S_BLADES_IN			=	211	; //	 - the sound of blades drawn back in
const byte 	S_GUN_PWRDOWN		=	240	; //	 - gun powerdown sound  // C_GUN_OFF,       // zB long press: one shot from the gun
	
const byte 	S_FLD_SPEAKS		=	5	; //	- speaks
const byte 	S_ANYTIME			=	1	; //	 - anytime     // C_SND_ANYTIME     // jE dbclick: play "anytime"
const byte 	S_OVERHERE			=	2	; //	 - over here, over here  //
	
const byte 	S_FLD_LAUGHS		=	6	; //	- laughs
const byte 	S_LAUGHS			=	1	; //	 - laugh      // C_SND_LAUGH,      // jW dbclick: play laugh

const byte 	S_FLD_TESTING		=	99	; //	- system sounds
const byte 	S_TEST_INIT			=	1	; //	 - Initiating components testing
const byte 	S_TEST_PASSED		=	2	; //	 - passed
const byte 	S_TEST_FAILED		=	3	; //	 - failed
const byte 	S_TEST_NOT_CONNECT	=	4	; //	 - unable to connect
const byte 	S_TEST_VOLUME		=	5	; //	 - volume
const byte 	S_TEST_FINISHED 	=	6	; //	 - testing sequence completed. have a nice day.
const byte 	S_TEST_SHRT_BEEP	=	10	; //	 - short beep
const byte 	S_TEST_SHRT_DBEEP	=	11	; //	 - short double-beep
const byte 	S_TEST_HP_BEEP		=	15	; //	 - short high pitch beep
const byte 	S_TEST_LP_BEEP		=	16	; //	 - long low pitch beep
const byte 	S_TEST_ERROR		=	19	; //	 - error

const byte 	S_TRACK_IMU			=	30	; //	 - swiching to inertia tracking mode
const byte 	S_TRACK_MARG		=	31	; //	 - swiching to magnetic tracking mode
const byte 	S_TRACK_CAL1		=	32	; //	 - Please perform compass calibration routine number one after the beep
const byte 	S_TRACK_CAL2		=	33	; //	 - Please perform compass calibration routine number two after the beep
const byte 	S_TRACK_CALS		=	34	; //	 - Compass calibration success
const byte 	S_TRACK_CALF		=	35	; //	 - Compass calibration failed

const byte 	S_TEST_MIN			=	71	; //	 - minimum
const byte 	S_TEST_MED			=	72	; //	 - medium
const byte 	S_TEST_MAX			=	73	; //	 - maximum

const byte 	S_TEST_COUNT		=	80	; //	- counting from 1 to 30
	
const byte 	S_TEST_MAIN_SND		=	100	; //	 - main sound system
const byte 	S_TEST_HEAD_CTRL	=	101	; //	 - head microcontroller
const byte 	S_TEST_HEAD_SND		=	102	; //	 - head sound system
const byte 	S_TEST_HEAD_GYRO	=	103	; //	 - head gyroscope
const byte 	S_TEST_HEAD_LED 	=	104	; //	 - head L.E.D's

const byte 	S_TEST_GUN_PWM		=	200	; //	 - plasma gun PWM board
const byte 	S_TEST_GUN_HSERVO	=	201	; //	 - plasma gun horizontal servo motor
const byte 	S_TEST_GUN_VSERVO	=	202	; //	 - plasma gun vertical servo motor
const byte 	S_TEST_GUN_3CLED	=	203	; //	 - plasma gun three color LED
const byte 	S_TEST_GUN_LASER	=	204	; //	 - plasma gun laser pointer
const byte 	S_TEST_GUN_LEDS		=	205	; //	 - plasma gun front LED assembly
const byte 	S_TEST_GUN_SOUND	=	206	; //	 - plasma gun blasting sounds
const byte 	S_TEST_GUN_GYRO		=	207	; //	 - plasma gun gyroscope
	
const byte 	S_TEST_HAND_WAIT	=	220	; //	 - waiting for gauntlet heartbeat
const byte 	S_TEST_HAND_CONNECT	=	221	; //	 - gauntlet is connected
const byte 	S_TEST_HAND_MISSING	=	222	; //	 - gauntlet not detected. Off or out of range
	
const byte 	S_TEST_THEME		=	254	; //	 - Predator theme
const byte 	S_TEST_GUMMYBEAR	=	255	; //	 - I am a gummy bear song

#endif //_PREDATOR_SOUNDS_H