/*
 *  C Implementation: ics9lpr363
 *
 * Description: ICS9LPR363(DGLF) for Asus A8E/A8S/A8J (maybe A8F/F8S/N80/N81)
 *
 *
 * Author: wlkmanist <vlad.king5555@mail.ru>, (C) 2023
 *         t.me/wlkmanist
 *
 * Copyright: See COPYING file that comes with this distribution
 *
 */
#include "i2c.h"
#include <math.h>

#ifdef DEBUG
#include <stdio.h>
#endif

#define BYTECOUNT 21
#define CMD 0x00

static int FSBIndex = 0;
static const unsigned int FSB_Min = 95;
static const unsigned int FSB_Max = 368;

int ics9lpr363_CheckFSB(int fsb, float *ram, float *pci, float *agp)
{
	if(ram)
		*ram = -1.0f;
	if(pci)
		*pci = -1.0f;
	if(agp)
		*agp = -1.0f;

	if(fsb <= FSB_Max && fsb >= FSB_Min)
		return 0;

	return -1;
}

int ics9lpr363_SetFSB(int fsb)
{
	int file, res;
	unsigned char buf[32];
	unsigned int tgt_step = 0; 	// 0-767
	unsigned int pll_m = 8; 	// 94.5-369MHz range

	file = i2c_open();
	if(file < 0)
		return -1;

	res = i2c_smbus_read_block_data(file, CMD, buf);
	if(res != BYTECOUNT)
	{
#ifdef DEBUG
		printf("SetFSB DEBUG: %i (should be %i) bytes read : ", res, BYTECOUNT);
		for(int i=0; i<res; i++)
			printf("%02X ", buf[i]);
		printf("\n");
#endif /* DEBUG */
		i2c_close();
		return -1;
	}
#ifdef DEBUG
	else
	{
		printf("SetFSB DEBUG: %i bytes read : ", res);
		for(int i=0; i<res; i++)
			printf("%02X ", buf[i]);
		printf("\n");
	}
#endif /* DEBUG */

	tgt_step = ((double)fsb - 94.5) / 0.35789474;

	if (tgt_step > 0x1FF)
		buf[0x0B] = 0xC0;
	else if (tgt_step > 0xFF)
		buf[0x0B] = 0x40;
	else
		buf[0x0B] = 0x80;
	buf[0x0B] |= (pll_m & 0x3F);
	buf[0x0C] = tgt_step & 0xFF;

	res = i2c_smbus_write_block_data(file, CMD, BYTECOUNT, buf);
	i2c_close();

 if(res < 0)
	 return -1;
#ifdef DEBUG
  else
		printf("SetFSB DEBUG: %i bytes written : ", BYTECOUNT);
  for(int i=0; i<BYTECOUNT; i++)
		printf("%02X ", buf[i]);
  printf("\n");
#endif /* DEBUG */

	return 0;
}

int ics9lpr363_GetFSB()
{
	int file, res;
	unsigned char buf[32];
	int ret = 0;

	file = i2c_open();
	if(file < 0)
		return -1;
	res = i2c_smbus_read_block_data(file, CMD, buf);
	i2c_close();

	if(res < 0) return -1;
#ifdef DEBUG
	else
	{
		printf("GetFSB DEBUG: %i bytes read : ", res);
		for(int i=0; i<res; i++)
			printf("%02X ", buf[i]);
		printf("\n");
	}
#endif /* DEBUG */

	int step_mult;

	if ((buf[0x0B] & 0xC0) == 0xC0)
		step_mult = 2;
	else if ((buf[0x0B] & 0xC0)== 0x40)
		step_mult = 1;
	else
		step_mult = 0;

	ret = (int)buf[0x0C] + (0x100 * step_mult); // Frequency step
	ret = (int)((double)ret * 0.35789474 + 94.5 + 0.5); // Real freq
	return ret;
}

int ics9lpr363_GetFirstFSB()
{
	FSBIndex = FSB_Min;

	return FSBIndex;
}

int ics9lpr363_GetNextFSB()
{
	FSBIndex++;

	if(FSBIndex <= FSB_Max)
		return FSBIndex;

	return -1;
}
