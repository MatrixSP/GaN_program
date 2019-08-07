#include "include.h"

void FTM_COMP_reinit(FTMn_e ftmn, FTM_CHn_e ch, uint32 freq, uint32 duty)
{
	FTM_PWM_init(ftmn, ch, freq, duty);




}