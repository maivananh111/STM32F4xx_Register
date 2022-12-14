/*
 * LoRa_API.h
 *
 *  Created on: Dec 14, 2022
 *      Author: anh
 */

#ifndef LORA_API_H_
#define LORA_API_H_


#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "LORA.h"
//#include "bits/stdc++.h"

#define LORA_CMD_GETID "GETID"
#define LORA_CMD_GETDATA "GETDATA"
#define LORA_CMD_CONTROL "CONTROL"
#define LORA_CMD_NODATA "*"


#define COUNT_NODE 10



typedef enum{
	Node_Command_GetID,
	Node_Command_Data,
	Node_Command_Control,
} Command_t;


typedef struct{
	char *cmd = NULL;
	char *data = NULL;
	Command_t command = Node_Command_Data;
} Packet_t;


extern uint8_t Node_ID[COUNT_NODE];
extern bool Node_Recieved[COUNT_NODE];
extern char Init_id[2];
extern bool Timeout;
extern uint32_t Timing_init_node;

void Init_Node(void);
void Get_Data(void);

void LoRa_AP_Init(LoRa *lora);
void LoRaTransmit(const char *command, const char *fomat, ...);

int FindChar(char *str, char ch);
char *GetCmd(char *str);
char *GetData(char *str);

Packet_t LoRa_ParseData(char *data);

void LoRa_RequestData(void);

#endif /* LORA_API_H_ */
