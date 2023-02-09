#ifndef __MK_TCP_PROTOCOL_H__
#define __MK_TCP_PROTOCOL_H__
#include <memory>
#include <stdlib.h>
#include<stdio.h>	
typedef struct CMDPROTOCOL
{
	char cmd[6];// temp stack
	unsigned char data[14];
	/*
	0:data[5] -> CMD (5digits)
	5:length[8] -> data size (8digits)
	13:status[1] -> (1digits)
	14
    e.g: CMD->img, datasize->640*480
	*/
	CMDPROTOCOL()
	{
		reset();
	}
	void reset()
	{
		memset(cmd, 0, 6);
		memset(data, 0, 14);
	}
	void setCMD(const char * cmd)
	{
        reset();
		memcpy(data, cmd, 5);
	}
	char * getCMD()
	{
		memcpy(cmd, data, 5);
		cmd[5] = '\0';
		return cmd;

	}
	bool isSameCmd(const char *command_string) {
		if(data[13]=='1' && strncmp((const char*)data, command_string, 5)==0) return true;
		return false;
	}
    int getPacketSize()
	{
		return (sizeof(data));
	}
	void setData(int L)
	{
        
		snprintf((char *)&data[5], 8, "%d", L);
		/*data[3] = L;
		data[4] = L >> 8;
		data[5] = L >> 16;
		data[6] = L >> 24;*/

	}
	void setData(float L)
	{
		snprintf((char *)&data[5], 8, "%f", L);
	}
	int getDatai()
	{
		char tmp[10];
		strncpy(tmp, (char *)&data[5], 8);
		tmp[7] = '\0';
		return (atoi(tmp));
		//return(data[6] << 24 | data[5] << 16 | data[4] << 8 | data[3]);
	}
	int getDataf()
	{
		char tmp[10];
		strncpy(tmp, (char *)&data[5], 8);
		tmp[8] = '\0';
		return (atof(tmp));
		//return(data[6] << 24 | data[5] << 16 | data[4] << 8 | data[3]);
	}

	void setStatus(char S)
	{
		data[13] = S;
	}
	char  getStatus()
	{
		return data[13];
	}
}cmdProtocol;

#endif //__MK_TCP_PROTOCOL_H__