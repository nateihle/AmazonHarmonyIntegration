//1st order linear interpolator coefficients for 3X polyphase interpolation
//filter
//-- x(k)   = 1*x(k) +   0*x(k+3)
//-- x(k+1) = 1/3*x(k) + 2/3*x(k+3)
//-- x(k+2) = 2/3*x(k) + 1/3*x(k+3)
#define  H_INTERP_1ORDER_3X   \
{ \
    1.00000000, \
    0.33333334, \
    0.66666667, \
    0.00000000, \
    0.66666667, \
    0.33333334  \
}

//1st order linear interpolator coefficients for 2X polyphase interpolation
//filter
//-- x(k)   = 1*x(k) +   0*x(k+3)
//-- x(k+1) = 1/3*x(k) + 2/3*x(k+3)
//-- x(k+2) = 2/3*x(k) + 1/3*x(k+3)
#define  H_INTERP_1ORDER_2X   \
{ \
    1.00000000, \
    0.50000000, \
    0.00000000, \
    0.50000000  \
}

//1st order linear interpolation coefficients for 6X polyphase interpolation
//--Polyphase filter coefficients
// {1, 1/6, 2/6, ... 5/6, 0, 5/6, 4/6 ... 1/6}
#define H_INTERP_1ORDER_6X  \
{ \
    1.00000000, \
    0.16666667, \
    0.33333333, \
    0.50000000, \
    0.66666667, \
    0.83333333, \
    0.00000000, \
    0.83333333, \
    0.66666667, \
    0.50000000, \
    0.33333333, \
    0.16666667, \
}

// {0, 1/3, 2/3, 1, 2/3, 1/3 }
// -- using newer to older delay line value indices
#define H_INTERP_1ORDER_3X_Q1D31  \
{ \
    0x00000000, \
    0x2AAAAAAA, \
    0x55555555, \
    MAXFRACT32, \
    0x55555555, \
    0x2AAAAAAA, \
}

// {0, 1/6, 2/6, ... 5/6, 1, 5/6, 4/6 ... 1/6}
// -- using newer to older delay line value indices
#define H_INTERP_1ORDER_6X_Q1D31  \
{ \
    0x00000000, \
    0x15555555, \
    0x2AAAAAAA, \
    0x40000000, \
    0x55555555, \
    0x6AAAAAAA, \
    MAXFRACT32, \
    0x6AAAAAAA, \
    0x55555555, \
    0x40000000, \
    0x2AAAAAAA, \
    0x15555555 \
}

// Python firls Design with cutoff around 5000Hz and 
// >20dB stop-band after 8Khz
// ['0.028467651', '0.066874938', '0.105958578', '0.139625284', 
//  '0.162382326', '0.170421608', '0.162382326', '0.139625284', 
//  '0.105958578', '0.066874938', '0.028467651']
#define H_INTERP_1ORDER_LS_6X_Q1D31  \
{ \
    0x03a4d3f8, \
    0x088f5ba3, \
    0x0d900cf9, \
    0x11df3dc6, \
    0x14c8f1ae, \
    0x15d06011, \
    0x14c8f1ae, \
    0x11df3dc6, \
    0x0d900cf9, \
    0x088f5ba3, \
    0x03a4d3f8, \
    0x00000000  \
}


// Python firls Design with cutoff around 6000Hz and 
// >20dB stop-band after 8Khz
// ['0.109533111', '0.155300439', '0.174099941', '0.155300439', '0.109533111']
#define H_INTERP_1ORDER_LS_3X_Q1D31  \
{ \
    0x0e052e55, \
    0x13e0e281, \
    0x1648e829, \
    0x13e0e281, \
    0x0e052e55 \
}

//['-0.023557377', '-0.071837256', '-0.035379305', '0.112073145', 
//  '0.290993789', '0.371691733', '0.290993789', '0.112073145', 
//  '-0.035379305', '-0.071837256', '-0.023557377']
#define H_INTERP_3ORDER_LS_3X_Q1D31  \
{ \
    0xfbed08aa, \
    0xfd2d87f1, \
    0x04fb4da5, \
    0x1224d722, \
    0x1ed44219, \
    0x2416571a, \
    0x1ed44219, \
    0x1224d722, \
    0x04fb4da5, \
    0xfd2d87f1, \
    0xfbed08aa \
}

// {1, 1, 1, 1, 1, 1}
#define  H_INTERP_0ORDER_6X_Q1D31  \
{ \
    MAXFRACT32, \
    MAXFRACT32, \
    MAXFRACT32, \
    MAXFRACT32, \
    MAXFRACT32, \
    MAXFRACT32 \
}

// {1, 1, 1}
#define  H_INTERP_0ORDER_3X_Q1D31  \
{ \
    MAXFRACT32, \
    MAXFRACT32, \
    MAXFRACT32 \
}

