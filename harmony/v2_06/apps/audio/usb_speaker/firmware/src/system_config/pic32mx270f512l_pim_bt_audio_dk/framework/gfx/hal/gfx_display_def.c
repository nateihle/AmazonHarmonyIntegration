#include "gfx/hal/inc/gfx_display.h"
#include "gfx/hal/inc/gfx_common.h"


GFX_DisplayInfo GFX_DisplayInfoList[] =
{
    {
	    "",  // description
		GFX_COLOR_MODE_RGB_565,  // color mode
		{
			0,  // x position (always 0)
			0,  // y position (always 0)
			176,  // display width
			220, // display height
		},
		{
		    8,  // data bus width
		    {
				42,  // horizontal pulse width
				2,  // horizontal back porch
				2,  // horizontal front porch
		    },
		    {
				10,  // vertical pulse width
				2,  // vertical back porch
				2,  // vertical front porch
		    },
			1,  // inverted left shift
		},
	},
};