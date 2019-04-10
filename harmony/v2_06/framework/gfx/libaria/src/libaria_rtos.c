#include "gfx/libaria/libaria.h"
#include "gfx/libaria/libaria_rtos.h"
#include "gfx/libaria/inc/libaria_context.h"
#include "gfx/libaria/inc/libaria_context_rtos.h"

void laUpdate_RTOS(laBool fullBlock, uint32_t dt)
{
    laContext_Update_RTOS(fullBlock, dt);
}