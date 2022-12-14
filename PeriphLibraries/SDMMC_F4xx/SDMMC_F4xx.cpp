/*
 * SDMMC_F4xx.cpp
 *
 *  Created on: Dec 11, 2022
 *      Author: anh
 */
#include "PERIPH_USED.h"
#ifdef ENABLE_SDMMC

#include "SDMMC_F4xx.h"
#include "SDMMC_Def.h"
#include "RCC_F4xx.h"
#include "GPIO_F4xx.h"
#include "PERIPH_STATUS.h"






static SDMMC_Config_t *_conf;

typedef struct{
	uint32_t Argument;
	uint32_t CmdIndex;
	uint32_t Response;
	uint32_t CPSM;
	uint32_t WaitIT;
}SDMMC_command_t;


static void SDMMC_SendCommand(SDMMC_command_t cmd);
static Status_t SDMMC_GetCmdError(void);

static uint32_t SDMMC_GetCmdResp1(uint8_t Cmd);
static uint32_t SDMMC_GetCmdResp2(void);
static uint32_t SDMMC_GetCmdResp3(void);
static uint32_t SDMMC_GetCmdResp6(uint8_t Cmd, uint16_t *pRCA);
static uint32_t SDMMC_GetCmdResp7(void);

static Status_t SDMMC_GotoIdleState(void);
static Status_t SDMMC_OperatingCondition(void);

static Status_t SD_PowerOn(void);


static void SDMMC_SendCommand(SDMMC_command_t cmd){
	SDIO -> ARG = cmd.Argument;
	SDIO -> CMD = cmd.CmdIndex | cmd.Response | cmd.WaitIT | cmd.CPSM;
}

/*
static uint32_t SDMMC_GetCmdResp1(uint8_t Cmd){
	return OKE;
}

static uint32_t SDMMC_GetCmdResp2(void){
	return OKE;
}


static uint32_t SDMMC_GetCmdResp3(void){
	return OKE;
}

static uint32_t SDMMC_GetCmdResp6(uint8_t Cmd, uint16_t *pRCA){
	return OKE;
}

static uint32_t SDMMC_GetCmdResp7(void){
	__IO uint32_t sta_reg;
	__IO uint32_t tick = gettick();

	while(((sta_reg & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) == 0U) || ((sta_reg & SDIO_FLAG_CMDACT) != 0U )){
		if(gettick() - tick > 50000U) return SDMMC_ERROR_TIMEOUT;

		sta_reg = SDIO -> STA;
	}

	if(__SDIO_GET_FLAG(SDIOx, SDIO_FLAG_CTIMEOUT))
	{
	__SDIO_CLEAR_FLAG(SDIOx, SDIO_FLAG_CTIMEOUT);

	return SDMMC_ERROR_CMD_RSP_TIMEOUT;oke
	}
	else if(__SDIO_GET_FLAG(SDIOx, SDIO_FLAG_CCRCFAIL))
	{
	__SDIO_CLEAR_FLAG(SDIOx, SDIO_FLAG_CCRCFAIL);

	return SDMMC_ERROR_CMD_CRC_FAIL;
	}
	else
	{
	}

	if(__SDIO_GET_FLAG(SDIOx, SDIO_FLAG_CMDREND))
	{
	__SDIO_CLEAR_FLAG(SDIOx, SDIO_FLAG_CMDREND);
	}

	return SDMMC_ERROR_NONE;
}
*/
static Status_t SDMMC_GetCmdError(void){
	Status_t sta = OKE;
	__IO uint32_t tick = gettick();
	while(!(SDIO -> STA & SDIO_STA_CMDSENT)){
		if(gettick() - tick > 5000){
			sta = TIMEOUT;
			return sta;
		}
	}
	SDIO -> ICR = ((uint32_t)(SDIO_STA_CCRCFAIL | SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CMDSENT));

	return sta;
}

static Status_t SDMMC_GotoIdleState(void){
	Status_t sta = OKE;
	SDMMC_command_t cmd = {
		.Argument = 0U,
		.CmdIndex = SDMMC_CMD_GO_IDLE_STATE,
		.Response = SDIO_RESPONSE_NO,
		.CPSM     = SDIO_CPSM_ENABLE,
		.WaitIT   = SDIO_WAIT_NO,
	};

	SDMMC_SendCommand(cmd);
	sta = SDMMC_GetCmdError();

	return sta;
}

static Status_t SDMMC_OperatingCondition(void){
	Status_t sta = OKE;
	SDMMC_command_t cmd = {
		.Argument = SDMMC_CHECK_PATTERN,
		.CmdIndex = SDMMC_CMD_HS_SEND_EXT_CSD, // SDMMC_CMD_GO_IDLE_STATE
		.Response = SDIO_RESPONSE_SHORT,
		.CPSM     = SDIO_CPSM_ENABLE,
		.WaitIT   = SDIO_WAIT_NO,
	};

	SDMMC_SendCommand(cmd);
//	sta = SDMMC_GetCmdResp7();

	return sta;
}
// HÀM NÀY CHƯA XONG /////////////////////////////////////////////////////////////////////////////////////////////////////////
static Status_t SD_PowerOn(void){
	Status_t sta = OKE;
	SDMMC_command_t cmd = {
		.Argument = 0U,
		.CmdIndex = 0U, // SDMMC_CMD_GO_IDLE_STATE
		.Response = 0U,
		.CPSM     = SDIO_CMD_CPSMEN,
		.WaitIT   = 0U,
	};
	/* CMD0: Goto idle state */
	SDMMC_SendCommand(cmd);
	sta = SDMMC_GetCmdError();
	if(sta != OKE) return sta;

	/* CMD8:  */


	return sta;
}


Result_t SDMMC_Init(SDMMC_Config_t *conf){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);
	__IO uint32_t qbus_freq = 0, tmpreg = 0;
	_conf = conf;

	/* SDIO MSP Init */
	GPIOC_CLOCKENABLE();
	GPIOD_CLOCKENABLE();

	RCC -> APB2ENR |= RCC_APB2ENR_SDIOEN;
	qbus_freq = RCC_GetBusFreq(QBUS);
	if(qbus_freq > 48000000U){
		Set_Line(&res, __LINE__);
		return res;
	}
	__IO uint32_t CLK_DIV = ((qbus_freq / 400000U) / (_conf -> sdmmc_clockdiv)) - 2;

	GPIO_AlternateFunction(GPIOC, 8,  AF12_FSMC_SDIO_USB); GPIO_AF_Type(GPIOC, 8,   GPIO_OUTPUT_PUSHPULL);
	GPIO_AlternateFunction(GPIOC, 9,  AF12_FSMC_SDIO_USB); GPIO_AF_Type(GPIOC, 9,   GPIO_OUTPUT_PUSHPULL);
	GPIO_AlternateFunction(GPIOC, 10, AF12_FSMC_SDIO_USB); GPIO_AF_Type(GPIOC, 10,  GPIO_OUTPUT_PUSHPULL);
	GPIO_AlternateFunction(GPIOC, 11, AF12_FSMC_SDIO_USB); GPIO_AF_Type(GPIOC, 11,  GPIO_OUTPUT_PUSHPULL);
	GPIO_AlternateFunction(GPIOC, 12, AF12_FSMC_SDIO_USB); GPIO_AF_Type(GPIOC, 12,  GPIO_OUTPUT_PUSHPULL);
	GPIO_AlternateFunction(GPIOD, 2,  AF12_FSMC_SDIO_USB); GPIO_AF_Type(GPIOD, 2,   GPIO_OUTPUT_PUSHPULL);

	/* SDIO CLOCK CONFIG */
	tmpreg = SDIO -> CLKCR;
	tmpreg |= (_conf -> sdmmc_clockedge) | (_conf -> sdmmc_widebus) | (CLK_DIV << SDIO_CLKCR_CLKDIV_Pos);
	tmpreg &=~ (SDIO_CLKCR_BYPASS | SDIO_CLKCR_HWFC_EN | SDIO_CLKCR_PWRSAV);
	SDIO -> CLKCR = tmpreg;

	SDIO -> CLKCR &=~ SDIO_CLKCR_CLKEN;
	SDIO -> POWER = SDIO_POWER_PWRCTRL;
	SDIO -> CLKCR |= SDIO_CLKCR_CLKEN;

	delay_ms(5);
	// HÀM NÀY CHƯA XONG /////////////////////////////////////////////////////////////////////////////////////////////////////////
	SD_PowerOn();


	return res;
}
































#endif

