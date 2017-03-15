/*
 * userinput.c
 *
 *  Created on: Mar 14, 2017
 *      Author: compi
 */
#include <string.h>
#include <limits.h>
#include <cardetector_common/userinput.h>
#include <stm32plus/usart.h>
#include <config.h>
#include <main.h>

//////////////////////////////////////////////////////////////////////////////
typedef struct
{
	const uint8_t	len;
	const char 		*cmd;
} CmdDesc;

const CmdDesc	g_commands[] =
{
	{ 4, "show" },	//0
	{ 4, "save" },	//1
	{ 5, "debug" },	//2
	{ 5, "mccnt" },	//3
	{ 5, "thdiv" },	//4
	{ 4, "tlim" }, //5
	{ 5, "shift" },	//6
	{ 0, NULL }
};

const CmdDesc	g_shifts[] =
{
	{ 5, "below" },	//0
	{ 4, "base" },	//1
	{ 5, "above" },	//2
	{ 4, "tout" },	//3
	{ 3, "sum" },	//4
	{ 0, NULL }
};

//////////////////////////////////////////////////////////////////////////////
extern LIVECONFIG	g_config;

const uint8_t	g_shiftDisps[] = { 0, 1, 2, 4 };

//////////////////////////////////////////////////////////////////////////////
char const * FindWordEnd(char const *buffer)
{
	uint8_t	u = *buffer;
	while(u && u != ' ' && u != '\t' && u != '=')
		u = *++buffer;
	return buffer;
}

//////////////////////////////////////////////////////////////////////////////
char const *SkipWhitespaces(char const *buffer)
{
	uint8_t	u = *buffer;
	while(u && (u == ' ' || u == '\t' || u == '='))
		u = *++buffer;
	return buffer;
}

//////////////////////////////////////////////////////////////////////////////
int8_t FindCommand(const CmdDesc *cmdtable, char const *buffer, uint8_t len)
{
	int8_t	ret = 0;
	while(cmdtable->len) {
		if(cmdtable->len == len && !strncmp(cmdtable->cmd, buffer, (size_t)len))
			return ret;
		++ret;
		++cmdtable;
	}
	return -1;
}

//////////////////////////////////////////////////////////////////////////////
char ConvertDigit( char c, uint8_t decimal )
{
	if( decimal )
		return (c >= '0' && c <= '9') ? c - '0' : -1;
	else {
		if( c >= '0' && c <= '9' ) return c - '0';
		if( c >= 'a' && c <= 'f' ) return c - 'a' + 10;
		if( c >= 'A' && c <= 'F' ) return c - 'A' + 10;
		return -1;
	}
}


//////////////////////////////////////////////////////////////////////////////
int GetIntParam( char const *input, uint8_t len, uint8_t decimal, uint8_t acceptneg )
{
	int	retval = 0;
	char	converted;
	uint8_t	found = 0;
	uint8_t	negative = 0;
	char const *tmp = input;

	while( tmp - input < len ) {
		converted = ConvertDigit(*tmp, decimal);
		if(converted == (char)-1) {
			if(!retval) {
				if(*tmp == 'x' || *tmp == 'X') {
					decimal = 0;
					converted = 0;
				} else if(*tmp == '-') {
					negative = 1;
					converted = 0;
				} else break;
			} else break;
		}
		retval *=  decimal ? 10 : 16;
		retval += converted;
		found = 1;
		++tmp;
	}
	return found ? (negative ? 0 - retval : retval ) : (acceptneg ? INT_MIN : -1);
}

//////////////////////////////////////////////////////////////////////////////
void ShowParam(char const *name, int16_t value)
{
	UsartSendStr(name, 1);
	UsartSendStr(": ", 1);
	UsartSendInt(value, 0, 1);
	UsartSendStr("\r\n", 1);
}

//////////////////////////////////////////////////////////////////////////////
void ShowParams(LIVECONFIG *config)
{
	uint8_t u;

	ShowParam(g_commands[3].cmd, config->mccount);
	ShowParam(g_commands[4].cmd, config->thdiv);
	ShowParam(g_commands[5].cmd, config->tlimit);
	UsartSendStr("\r\nSHIFTS:\r\n", 1);
	ShowParam(g_shifts[4].cmd, config->sumshift);
	for(u = 0; u < sizeof(g_shiftDisps) / sizeof(g_shiftDisps[0]); ++u) {
		ShowParam(
				g_shifts[u].cmd,
				config->shifts[g_shiftDisps[u]]
		);
	}
	UsartSendStr("\r\n", 1);

}

//////////////////////////////////////////////////////////////////////////////
void ProcessInput(LIVECONFIG *config, char const *buffer)
{
	int	InVal;
	char const *next = FindWordEnd(buffer);

	uint8_t	CmdID = FindCommand(g_commands, buffer, next - buffer);
	uint8_t ShiftID;

	buffer = SkipWhitespaces(next);
	next = FindWordEnd(buffer);

	switch(CmdID)
	{
	case 0:		//show
		ShowParams(config);
		break;

	case 1:		//save
		I2cEEPROM_Write(&g_eeprom, EESTART, config, sizeof(*config));
		break;

	case 2:		//debug
		if((InVal = GetIntParam(buffer, next - buffer, 1, 0)) >= 0)
			g_debug = InVal != 0;
		else g_debug = !g_debug;
		break;

	case 3:		//mccnt
		if((InVal = GetIntParam(buffer, next - buffer, 1, 0)) >= 0)
			config->mccount = InVal;
		break;

	case 4:		//thdiv
		if((InVal = GetIntParam(buffer, next - buffer, 1, 0)) >= 0)
			config->thdiv = InVal;
		break;

	case 5:		//tlim
		if((InVal = GetIntParam(buffer, next - buffer, 1, 0)) >= 0)
			config->tlimit = InVal;
		break;

	case 6:		//shift
		ShiftID = FindCommand(g_shifts, buffer, next - buffer);
		buffer = SkipWhitespaces(next);
		next = FindWordEnd(buffer);
		InVal = GetIntParam(buffer, next - buffer, 1, 0);
		if(InVal != INT_MIN) {
			if(ShiftID < 4)
				config->shifts[g_shiftDisps[ShiftID]] = InVal;
			else if(ShiftID == 4)
				config->sumshift = InVal;
		}
		break;
	}
}
