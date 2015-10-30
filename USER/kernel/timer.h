#ifndef _TIMER_H_
#define _TIMER_H_

struct tm_task
{
	unsigned long time_out;
	void *ptr;
	int (*pTask)(void *ptr);
};

#endif