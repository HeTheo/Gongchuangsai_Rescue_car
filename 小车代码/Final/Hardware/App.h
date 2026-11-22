#ifndef __APP_H
#define __APP_H

#include "Comm.h"

void Cmd_Dispatch(Comm_Frame_t *f);
void App_Poll(void);

#endif

