#include <stdio.h>

#include "CAN/CAN_Periodic.h"

//Interbal variables
periode_t* periode_;
uint16_t current_periode_ = 0;
uint16_t current_variable_ = 0;
uint8_t periodic_enable_ =0;
uint8_t do_periodic_ = 0;
uint16_t periode_ms_ = 0;

void initCanPeriodic(uint16_t periode_ms, periode_t* periodes)
{
	periode_ = periodes;
	periodic_enable_ = 1;
	current_variable_ = 0;
	periode_ms_ = periode_ms;
}

int sendSubPeriode(void)
{
	if(sendMessage(periode_->subperiodes[current_periode_].variables[current_variable_].id, *periode_->subperiodes[current_periode_].variables[current_variable_].data) == 0)
	{
		current_variable_ = current_variable_ + 1;
		if(current_variable_ >= periode_->subperiodes[current_periode_].nb_variables)
		{
			do_periodic_ = 0;
			current_variable_ = 0;
			return 0; //end of all variables in this subperiode
		}
	}

	return -1; //NOT the end of all variables in this subperiode
}

void CanCallback(uint64_t time_ms)
{
	if(periodic_enable_)
	{
		if(time_ms % periode_ms_==0)
			do_periodic_ = 1;
	}
}

void runCanPeriodic(void)
{
	if(do_periodic_)
	{
		if(sendSubPeriode() == 0)
		{
			current_periode_ = current_periode_ + 1;
			if(current_periode_ >= periode_->nb_subperiodes)
				current_periode_ = 0;
		}
	}
}
