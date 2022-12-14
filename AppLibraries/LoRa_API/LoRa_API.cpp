/*
 * LoRa_API.cpp
 *
 *  Created on: Dec 14, 2022
 *      Author: anh
 */
#include "LoRa_API.h"
#include "TIM_F4xx.h"


static LoRa *_lora;
extern TIM *timer3;

void LoRa_AP_Init(LoRa *lora){
	_lora = lora;
}
/*
uint32_t Node_ID[COUNT_NODE];
bool Node_Recieved[COUNT_NODE]={0};

char Init_id[2];
uint32_t Timing_init_node;
bool Timeout;
void Init_Node(void){
	for(int i=1; i<COUNT_NODE; i++){
		if(Node_ID[i]==0){
			LoRaTransmit(lora, "%d", i);
			Timing_init_node = gettick();
			sprintf(Init_id, "%02d", i);
		}
	}
}

void Get_Data(void){
	//******
	for(bool item: Node_Recieved){
		item = 0;
	}
	//******
	LoRaTransmit(lora, "DATA: 0");
	uint32_t max_time_respone = *std::max_element(Node_ID,Node_ID + COUNT_NODE);
	timer3->Set_AutoReload((max_time_respone + 2000) * 2);
	timer3->ResetCounter();
}
*/

int FindChar(char *str, char ch){
	char *chptr = strchr(str, ch);
	if(chptr == NULL) return -1;
	return (int)(chptr - str);
}

int GetID_in_data(char *str){
	int first_index = FindChar(str, ' ');
	char* ID;
	strncpy(ID, str, first_index);
	return atoi(ID);
}

char *GetCmd(char *str){
	int index = FindChar(str, ':');
    if(index > 0){
    	char *cmd = (char *)malloc(index + 1);
    	strncpy(cmd, str, index);
    	cmd[index] = '\0';
    	return cmd;
    }
    return NULL;
}

char *GetData(char *str){
	int index = FindChar(str, ':') + 2; //= 5
    if(index > 0){
    	uint8_t datalen = strlen(str) - index;
    	char *tmp = (char *)malloc(datalen + 1);
    	for(uint8_t i = index; i<strlen(str); i++) tmp[i - index] = str[i];
    	tmp[datalen] = '\0';
    	return tmp;
    }
    return NULL;
}

Packet_t LoRa_ParseData(char *data){
	Packet_t packet;
	packet.cmd = GetCmd(data);
	packet.data = GetData(data);

	if     (strcmp(packet.cmd, LORA_CMD_GETID) == 0) packet.command = Node_Command_GetID;
	else if(strcmp(packet.cmd, LORA_CMD_GETDATA) == 0)   packet.command = Node_Command_Data;
	else if(strcmp(packet.cmd, LORA_CMD_CONTROL) == 0) packet.command = Node_Command_Control;
	return packet;
}

void LoRaTransmit(const char *command, const char *fomat, ...){
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, fomat);

	uint16_t length = vsnprintf(Temp_buffer, 0, fomat, args);
	Temp_buffer = (char *)malloc(length * sizeof(char));
	vsprintf(Temp_buffer, fomat, args);
	va_end(args);

	char *Out_buffer = (char *)malloc((strlen(command) + 2 + length)*sizeof(char));
	sprintf(Out_buffer, "%s: %s", command, Temp_buffer);

	_lora -> beginPacket();
	_lora -> write((uint8_t*)Out_buffer, (size_t)strlen(Out_buffer) );
	_lora -> endPacket();
	_lora -> Receive(0);
}







