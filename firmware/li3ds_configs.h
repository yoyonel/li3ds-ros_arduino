#ifndef __LI3DS_CONFIGS__
#define __LI3DS_CONFIGS__

#define __DEBUG__
#define __LED_FLASH__

//
#define BAUD_RATE          (57600)
//
#define PPS_PIN				10
#define TX_PIN				4
#define RX_PIN				10
#define FLASH_PIN           11		// sortie pwm eclairage
#define CAM_PIN             8		// sortie commande photo aux cameras (toujours LOW, mais INPUT pour haute impedance et OUTPUT pour mettre a la masse)
#define CAM_BOOT_HALT       7
#define RUN_LED             6		// run arduino

#define UPDATE_RATE_PER_SECOND 1

// definitions flash 
#define MAX_FLASHLEVEL        255      // Niveau eclairement max (ie flash).
#define MIN_FLASHLEVEL        5        // Niveau eclairement eco.
#define FLASH_OFFLEVEL        0        // default FLASH ON.
#define STAB_FLASH_DELAY      2
#define FLASH_DELAY          10       // Extinction du flash apres prise de photo.

#define ROS_DELAY   100  // attente de 100ms dans la phase de temporisation de notre boucle temporelle
                        // => ~ 10Hz
// //
// #define BUZZER_PIN      5			//buzzer
// #define BUZZER_ON       255      	// Buzzer on .
// #define BUZZER_OFF      0        	// buzzer off.

// definitions cam
#define CAM_WRITE_DELAY		1		// delay apres ecriture sur pin de camlight
#define CAM_HALT_DELAY		500		// delay de 0,5s pour declencher un "halt", 
									// attention en dessous il ne se passe rien ou c'est compris comme un boot,
									// au dessus c'est un arret system violent (cas du plantage de l'os embarqué)
#define CAM_BOOT_DELAY		100		// delay de 0,1s pour declencher un boot

extern char ros_log[50];

#endif