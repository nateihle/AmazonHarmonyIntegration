package com.microchip.spp_voice_control;

import android.util.Log;

/**
 * Created by C16266 on 1/2/2018.
 */

public class ADPCMDecoder {
    class ADPCMState
    {
        int prevsample;
        int previndex;
    }
    private static ADPCMState adpcmState;

    static ADPCMDecoder adpcmDecoderInst;
    private ADPCMDecoder()
    {
        adpcmState = new ADPCMState();
        adpcmState.previndex = 0;
        adpcmState.prevsample = 0;
    }
    public static ADPCMDecoder getADPCMDecoderInst()
    {
        if(adpcmDecoderInst == null)
        {
            adpcmDecoderInst = new ADPCMDecoder();
        }
        return adpcmDecoderInst;
    }

    static final int[] indexTable={
        -1, -1, -1, -1, 2, 4, 6, 8,
        -1, -1, -1, -1, 2, 4, 6, 8,
    };

    static final int[] stepSizeTable = {
            7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
            19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
            50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
            130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
            337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
            876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
            2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
            5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
            15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
    };

    int adpcm_decode_frame(byte[] input, int inSize, byte[] output)
    {
        byte pChar;	// packed ADPCM character
        int i=0;
        while(i!=inSize)
        {
            pChar = input[i];
            // convert to bytes in little endian
            short lower = ADPCMDecodeSample((byte)((pChar)&0x0f));
            short upper = ADPCMDecodeSample((byte)((pChar>>4)&0x0f));

            output[4*i] = (byte)(lower&0xff);
            output[4*i+1] = (byte)((lower>>8)&0xff);
            output[4*i+2] = (byte)(upper&0xff);
            output[4*i+3] = (byte)((upper>>8)&0xff);

            i++;
        }

        // compress ratio 4:1
        return inSize*4;
    }

    static short ADPCMDecodeSample(byte code)
    {
        int step;		/* Quantizer step size */
        int predsample;		/* Output of ADPCM predictor */
        int diffq;		/* Dequantized predicted difference */
        int index;		/* Index into step size table */

    /* Restore previous values of predicted sample and quantizer step size index */
        predsample = adpcmState.prevsample;
        index = adpcmState.previndex;

    /* Find quantizer step size from lookup table using index */
        step = stepSizeTable[index];

    /* Inverse quantize the ADPCM code into a difference using the quantizer step size */
        diffq = step >> 3;
        if( (code&4) != 0 )
        {
            diffq += step;
        }
        if( (code&2) != 0 )
        {
            diffq += step >> 1;
        }
        if( (code&1) != 0 )
        {
            diffq += step >> 2;
        }

        /* Add the difference to the predicted sample */
        if( (code&8) != 0 )
        {
            predsample -= diffq;
        }
        else
        {
            predsample += diffq;
        }

        if(predsample>32767)
        {
            predsample=32767;
        }
        else if(predsample<-32768)
        {
            predsample=-32768;
        }



    /* Find new quantizer step size by adding the old index and a
      table lookup using the ADPCM code */
        index += indexTable[code];

    /* Check for overflow of the new quantizer step size index */
        if( index < 0 )
        {
            index = 0;
        }
        if( index > 88 )
        {
            index = 88;
        }


    /* Save predicted sample and quantizer step size index for next iteration */
        adpcmState.prevsample = predsample;
        adpcmState.previndex = index;

    /* Return the new speech sample */
        return (short)predsample;

    }

}
