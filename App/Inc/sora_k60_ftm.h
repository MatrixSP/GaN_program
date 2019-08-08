#ifndef __SORA_K60_FTM_H__
#define __SORA_K60_FTM_H__

typedef enum FTM_Gn_e
{
	FTM_G0,
	FTM_G1,
	FTM_G2,
	FTM_G3,
} FTM_Gn_e;

void FTM_COMP_init(FTMn_e ftmn, FTM_Gn_e gn, uint32 freq, uint32 duty, float deadtime);

#endif