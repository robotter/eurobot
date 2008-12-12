// EIRBOT 2005
// ToF
/** \file encoders_eirbot_config.h
 *  \brief configuration du module codeur
 *
 *  \todo il reste a impl�manter la version sur irq
 *
 *  \test a tester en version xil
 *
 * on peut configurer ici combien de codeurs seront utilis�s
 * et comment y acc�der (interface bus xilinx ou ports en irq)
 */

#ifndef _ENCODERS_EIRBOT_CONFIG_
#define _ENCODERS_EIRBOT_CONFIG_



/** mode de fonctionnement, au choix */
//#define CODEUR_MODE_IRQ
#define ENCODERS_MODE_XILINX


/** port utilis� pour la s�l�ction d'adresses dans le xilinx 
 * exemple, pour 4 codeurs, avec un port de s�l�ction de 2 bits sur le 	   portB, bits 5 et 6 :
 * #define CODEUR_SELEC_NITS_NUM 2
 * #define CODEUR_SELEC_BIT0 5
 * #define CODEUR_SELEC_PORT PORTB
 * #define CODEUR_SELEC_DDR  DDRB
*/

#define ENCODERS_NUMBER 4
#define ENCODERS_SELEC_NITS_NUM 3
#define ENCODERS_SELEC_BIT0 0
#define ENCODERS_SELEC_PORT PORTA
#define ENCODERS_SELEC_DDR  DDRA

/** d�finition du bus 8 bits utilis� dans le mode xilinx */
#define ENCODERS_PIN PINB
#define ENCODERS_DATA_NBBITS 8
#define ENCODERS_DATA_BIT0 0
#define ENCODERS_DATA_DDR DDRB



#endif


