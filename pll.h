//
// C++ Interface: pll
//
// Description: PLL IC functions
//
//
// Author: Nikolay Kislitsa <deusexbeer@gmail.com>, (C) 2013
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef PLL_H
#define PLL_H

#define PLL_MAKE_FUNCS(name) \
extern int name ## _SetFSB(int fsb); \
extern int name ## _GetFSB(); \
extern int name ## _CheckFSB(int fsb, float *ram, float *pci, float *agp); \
extern int name ## _GetFirstFSB(); \
extern int name ## _GetNextFSB(); \
extern int name ## _CheckTME();

#define PLL_MAKE_STRUCT(textname, name, tested) {textname, tested, name ## _SetFSB, name ## _GetFSB, name ## _CheckFSB, name ## _GetFirstFSB, name ## _GetNextFSB, NULL}

#define PLL_MAKE_STRUCT_TME(textname, name, tested) {textname, tested, name ## _SetFSB, name ## _GetFSB, name ## _CheckFSB, name ## _GetFirstFSB, name ## _GetNextFSB, name ## _CheckTME}

typedef enum
{
	Tested = 0x01,
	UnTested = 0x02,
	Testing = 0x04
} Test_t;

typedef struct
{
	char *name;
	Test_t flags;

	int (*SetFSB)(int fsb);
	int (*GetFSB)();
	int (*CheckFSB)(int fsb, float *ram, float *pci, float *agp);
	int (*GetFirstFSB)();
	int (*GetNextFSB)();
	int (*CheckTME)();
} PLL_t;

PLL_MAKE_FUNCS(ics9148_04)
PLL_MAKE_FUNCS(ics9148_26)
PLL_MAKE_FUNCS(ics9148_36)
PLL_MAKE_FUNCS(ics9148_58)
PLL_MAKE_FUNCS(ics9150_08)
PLL_MAKE_FUNCS(ics9248_39)
PLL_MAKE_FUNCS(ics9248_64)
PLL_MAKE_FUNCS(ics9248_110)
PLL_MAKE_FUNCS(ics9248_114)
PLL_MAKE_FUNCS(ics9248_141)
PLL_MAKE_FUNCS(ics9248_153)
PLL_MAKE_FUNCS(ics9248_199)
PLL_MAKE_FUNCS(ics9250_08)
PLL_MAKE_FUNCS(ics94215)
PLL_MAKE_FUNCS(ics94228)
PLL_MAKE_FUNCS(cy28331)
PLL_MAKE_FUNCS(ics9lprs477)
PLL_MAKE_FUNCS(slg8sp513)
PLL_MAKE_FUNCS(ics9lprs355)
PLL_MAKE_FUNCS(ics9lprs365)
PLL_MAKE_FUNCS(ics9lpr363)

const PLL_t const PLL[] =
{
	PLL_MAKE_STRUCT("ics9148-04", ics9148_04, UnTested),
	PLL_MAKE_STRUCT("ics9148-26", ics9148_26, UnTested),
	PLL_MAKE_STRUCT("ics9148-36", ics9148_36, UnTested),
	PLL_MAKE_STRUCT("ics9148-58", ics9148_58, UnTested),
	PLL_MAKE_STRUCT("ics9150-08", ics9150_08, UnTested),
	PLL_MAKE_STRUCT("ics9248-39", ics9248_39, UnTested),
	PLL_MAKE_STRUCT("ics9248-64", ics9248_64, UnTested),
	PLL_MAKE_STRUCT("ics9248-110", ics9248_110, Tested),
	PLL_MAKE_STRUCT("ics9248-114", ics9248_114, UnTested),
	PLL_MAKE_STRUCT("ics9248-141", ics9248_141, UnTested),
	PLL_MAKE_STRUCT("ics9248-153", ics9248_153, UnTested),
	PLL_MAKE_STRUCT("ics9248-199", ics9248_199, UnTested),
	PLL_MAKE_STRUCT("ics9250-08", ics9250_08, UnTested),
	PLL_MAKE_STRUCT("ics94215", ics94215, UnTested),
	PLL_MAKE_STRUCT("ics94228", ics94228, UnTested),
	PLL_MAKE_STRUCT("cy28331", cy28331, Testing),
	PLL_MAKE_STRUCT("ics9lprs477bkl", ics9lprs477, Tested),
	PLL_MAKE_STRUCT("slg8sp513", slg8sp513, Tested),
	PLL_MAKE_STRUCT_TME("ics9lprs355", ics9lprs355, UnTested),
	PLL_MAKE_STRUCT_TME("ics9lprs365", ics9lprs365, UnTested),
	PLL_MAKE_STRUCT("ics9lpr363", ics9lpr363, UnTested),
	{""}
};

#endif
