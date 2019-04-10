<#-- math_dsp_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_math_dsp_app_c_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_math_dsp_app_c_global_data>
</#macro>
<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_math_dsp_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_math_dsp_app_c_local_functions>

/******************************************************************************
  Function:
    static void ${APP_NAME?upper_case}_AES_CBC_Encrypt_Task (void)

   Remarks:
    Encrypts plain text using AEC CBC cipher.

*/
static void ${APP_NAME?upper_case}_MATH_DSP_Task(void)
{
    switch (${APP_NAME?lower_case}Data.mathDspStates)
    {
        default:

        case ${APP_NAME?upper_case}_MATH_DSP_STATE_START:
<#if ("CONFIG_DSP_COMPLEX_CONJ" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_ComplexDotProd32:

            int32c *res, result;
            int32c *input1, *input2;
            int32c test_complex_1 = {0x40000000,0x0CCCCCCC};
            //                       (0.5 + 0.1i)
            int32c test_complex_2 = {0x73333333,0xB3333334};
            //                       (0.9 - 0.6i)

            res=&result;
            input1=&test_complex_1;
            input2=&test_complex_2;

            DSP_ComplexDotProd32(input1, input2, res);
            // result = {0x31EB851E, 0xCE147AE3} = (0.39 - 0.39i)
        }

</#if>
<#if ("CONFIG_DSP_FFT" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_TransformFFT16:
            int log2N = 8;  // log2(256) = 8
            int fftSamples = 256;

            int16c  *fftDin;
            int16c  fftDout[fftSamples];
            int16c  scratch[fftSamples];
            int16c  fftCoefs[fftSamples];
            int16c *fftc = fftCoefs;

            // From an external producer. Placed here to allow compile and demo
            int16c fftin_8Khz_long_window16[fftSamples];

            DSP_TransformFFT16_setup(fftc, log2N);  // call setup function

            fftDin = fftin_8Khz_long_window16;  // get 256 point complex data

            DSP_TransformFFT16(fftDout, fftDin, fftc, scratch, log2N);
        }

</#if>
<#if ("CONFIG_DSP_FILTER_FIR" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_FilterFIR32:

#define TAPS 4
#define numPOINTS 256

            int filterN = numPOINTS;
            int filterK = TAPS;
            int filterScale = 1;  // scale output by 1/2^1 => output * 0.5
            int32_t FilterCoefs[TAPS*2] = {0x40000000, 0x20000000, 0x20000000, 0x20000000,
                                           0x40000000, 0x20000000, 0x20000000, 0x20000000};
            // note repeated filter coefs, A B C D A B C D
            //             0.5, 0.25, 0.25, 0.25, 0.5, 0.25, 0.25, 0.25

            int32_t __attribute__ ((aligned (32))) outFilterData[numPOINTS] = {0};
            int32_t __attribute__ ((aligned (32))) inFilterData[numPOINTS];
            int filterDelayLine[TAPS] = {0};

            //   put some data into input array, inFilterData, here //
            DSP_FilterFIR32(outFilterData, inFilterData, FilterCoefs,
                            filterDelayLine, filterN, filterK, filterScale);
        }

</#if>
<#if ("CONFIG_DSP_FILTER_IIRBQ" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_FilterIIRBQ16:

            PARM_EQUAL_FILTER *ptrFilterEQ;
            PARM_EQUAL_FILTER FilterEQ;
            uint16_t DataIn, DataOut;
            ptrFilterEQ = &FilterEQ;

            // 48KHz sampling; 1 KHz bandpass filter; Q=0.9
            //  divide by 2 and convert to Q15
            //   b0 = 0.06761171785499065
            //   b1 = 0
            //   b2 = -0.06761171785499065
            //   a1 = -1.848823142275648
            //   a2 = 0.8647765642900187

            // note all coefs are half value of original design, gain handled in algorithm
            ptrFilterEQ->b[0]=0x0453;       // feed forward b0 coef
            ptrFilterEQ->b[1]=0;            // feed forward b1 coef
            ptrFilterEQ->b[2]=0xFBAD;       // feed forward b2 coef

            // note all coefs are half value of original design, gain handled in algorithm
            // note subtract is handled in algorithm, so coefs go in at actual value
            ptrFilterEQ->a[0]=0x89AD;       // feedback a1 coef
            ptrFilterEQ->a[1]=0x3758;       // feedback a2 coef

            int i;
            uint16_t three_hundred_hz[256];  // for demo - external producer needed
            for (i=0;i<256;i++)
            {
                // *** get some input data here
                DataIn = three_hundred_hz[i];

                DataOut = DSP_FilterIIRBQ16(DataIn, ptrFilterEQ);
                // *** do something with the DataOut here
             }
        }

</#if>
<#if ("CONFIG_DSP_FILTER_LMS" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_FilterLMS16:

#define lmsTAPS 8

            int16_t lmsOut;
            int lmsTaps = lmsTAPS;

            int16_t __attribute__ ((aligned (32))) lmsCoefs[lmsTAPS] =
                 {0x5000, 0x4000,0x3000, 0x2000, 0x1000, 0x0000,0xF000, 0xE000};
            int16_t __attribute__ ((aligned (32))) lmsDelay[lmsTAPS]={0};
            int16_t *ptrLMSError;
            int16_t lmsError = 0x0200;
            int16_t inVal=0;

            int16_t refVal = 0x0CCC;  // some target value = 0.1
            int16_t lmsAdapt = 0x3000;

            ptrLMSError = &lmsError;

            int i;
            for (i=0;i<200;i++)
            {
                // get some input value here //
                if (i < 100)
                {
                    inVal = 0x4233;
                }
                else
                {
                    inVal = 0xCF10;
                }

                lmsOut = DSP_FilterLMS16(inVal, refVal, lmsCoefs, lmsDelay,
                                         ptrLMSError, lmsTaps, lmsAdapt);
            }
        }

</#if>
<#if ("CONFIG_DSP_MATRIX_TRANSPOSE" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_MatrixTranspose32:

#define ROW 3
#define COL 4

            matrix32 *resMat, *srcMat1;
            int32_t result[ROW*COL];

            int32_t matA[ROW*COL] = { 1,  2,  3,  4,
                                      5,  6,  7,  8,
                                     -1, -3, -5, -7};

            matrix32 mat, mat2;
            resMat=&mat;
            srcMat1=&mat2;

            srcMat1->row=ROW;
            srcMat1->col=COL;
            srcMat1->pMatrix=matA;

            resMat->row=COL;   // note the shift in columns and rows
            resMat->col=ROW;
            resMat->pMatrix=result;

            DSP_MatrixTranspose32(resMat, srcMat1);

            // result[] = matA(T)[] = {   1,  5, -1,
            //                            2,  6, -3,
            //                            3,  7, -5,
            //                            4,  8, -7}
        }

</#if>
<#if ("CONFIG_DSP_VECTOR_ABS" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_VectorAbs32:

            int32_t *pOutdata;
            int32_t __attribute__ ((aligned (32))) outVal[8];
            int32_t __attribute__ ((aligned (32))) inBufTest[16] =
            {-5, 2, -3, 4, -1, 0, -2, -8, -21, 21, 10, 100, 200, 127, -127, -2};
            int Num = 8;
            pOutdata = outVal;

            DSP_VectorAbs32(pOutdata, inBufTest, Num);

            // outVal[i] = {5,2,3,4,1,0,2,8}
        }

</#if>
<#if ("CONFIG_DSP_VECTOR_ADD" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_VectorAdd32:

            int32_t *pOutdata;
            int32_t __attribute__ ((aligned (32))) outVal[8];
            int32_t __attribute__ ((aligned (32))) inBufTest[16] =
                    {-5,2,-3,4,-1,0,-2,-8,-21,21,10,100, 200, 127,-127,-2};
            int32_t __attribute__ ((aligned (32))) inBuf2[16] =
                    { 1,2, 3,4, 5,6, 7, 8, 9, 10,-1,-100,-127,127,-7,  0};
            int Num = 8;
            pOutdata = outVal;

            DSP_VectorAdd32(pOutdata, inBufTest, inBuf2, Num);

            // outVal[i] = inBufTest[i] + inBuf2[i] = {-4,4,0,8,4,6,5,0}
        }

</#if>
<#if ("CONFIG_DSP_VECTOR_DOTP" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_VectorDotp16:

            int16_t __attribute__ ((aligned (32))) inBufMultA[8] =
               {0x7FFF, 0x8000, 0x7333, 0x6666, 0x1999, 0x4000, 0x7FFF, 0xB334};
            //      1,     -1,      0.9,    0.8,    0.2,   0.5,    1,     -0.6

            int16_t __attribute__ ((aligned (32))) inBufMultB[8] =
               {0x0CCD, 0x0CCD, 0x4000, 0xC000, 0xE667, 0x4000, 0x0000, 0x0CCD};
            //    0.1,    0.1,    0.5,   -0.5,   -0.2,   0.5,    0,      0.1

            int Num = 8;
            int scaleVal = 2;
            int16_t outScalar;

            outScalar = DSP_VectorDotp16(inBufMultA, inBufMultB, Num, scaleVal);

            // outScalar = 1/(2^scaleVal)*(inBufMultA[] dot inBufMultB[]) =
            //   (1/4) * (0.1 + -0.1 + 0.45 + -0.4 + -0.04 + 0.25 + 0 + -0.06) = 0.25 * 0.20 = 0.05
            //      = (int16_t)0x0666
        }

</#if>
<#if ("CONFIG_DSP_VECTOR_EXP" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_VectorExp:

            int expNum = 4;

            _Q16 __attribute__ ((aligned (32))) inExpVec[8] =
                {0x00010000, 0xffff0000,0x00020000,0x00030000,
                 0x00038000, 0x00040000,0xfffe0000,0x00058000};
            //    1.0,  -1.0,  2.0,  3.0,   3.5,   4.0,  -2.0,  5.5

            _Q16 __attribute__ ((aligned (32))) outExpVec[8] = {0};

            DSP_VectorExp(outExpVec, inExpVec, expNum);

            // outExpVec = 0x0002B7E1, 0x00005E2D, 0x00076399, 0x001415E6,
            //                      0,          0,          0,          0
            //                2.71828,    0.26787,     7.3891,    20.0855,
            //                      0,          0,          0,          0
        }

</#if>
<#if ("CONFIG_DSP_VECTOR_RMS" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_VectorRMS16:

            int16_t __attribute__ ((aligned (32))) vecRMSIn[8] =
                {0x1999, 0xD99A, 0x4000, 0x2666,0x1999,0x1999,0x2666, 0x3333};
            //      0.2,   -0.3,    0.5,    0.3,   0.2,   0.2,   0.3,    0.4

            int16_t RMSOut=0;
            int Nrms = 8;

            RMSOut = DSP_VectorRMS16(vecRMSIn, Nrms);
            // RMSOut = 0x287C (= 0.31628)
        }

</#if>
<#if ("CONFIG_DSP_VECTOR_SUMS" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_VectorSumSquares32:

            int32_t __attribute__ ((aligned (32))) inBufMultA[8] =
                {0x7FFFFFFF, 0x80000000, 0x73333333, 0x66666666,
                 0x19999999, 0x40000000, 0x00000000, 0xB3333334};
            //       1, -1, 0.9, 0.8,  0.2, 0.5, 1, -0.6

            int Num = 8;
            int scaleVal = 3;
            int32_t outScalar;

            outScalar = DSP_VectorSumSquares32(inBufMultA, Num, scaleVal);

            // outScalar = 1/(2^scaleVal)* sum(inBufMultA[i]^2) =
            //   (1/8) * (1 + 1 + 0.81 + 0.64 + 0.04 + 0.25 + 1 + 0.36) = 0.125 * 5.1 = 0.6375
            //      = (int32_t)0x51999999
        }

</#if>
<#if ("CONFIG_DSP_VECTOR_VARIANCE" + "${HCONFIG_APP_INSTANCE}")?eval>
        {
            // Function DSP_VectorVariance:
            int varN = 8;

            int32_t __attribute__ ((aligned (32))) inVarVec[8] =
                {0xE6666667, 0x40000000, 0x40000000, 0x0CCCCCCC,
                 0x00000000, 0x59999999, 0x20000000, 0xC0000000};
            //     -0.2, 0.5, 0.5, 0.1,  0, 0.7, 0.25, -0.5
            int32_t outVar = 0;

            outVar = DSP_VectorVariance(inVarVec, varN);
            // outVar = 0x1490D2A6 = 0.1606696
        }

</#if>
            // Finished - move to next state
            ${APP_NAME?lower_case}Data.mathDspStates = ${APP_NAME?upper_case}_MATH_DSP_STATE_DONE;
            break;

        case ${APP_NAME?upper_case}_MATH_DSP_STATE_DONE:
            break;
    }
}
</#macro>

<#--
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Initialize ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;
-->

<#macro macro_math_dsp_app_c_initialize>
</#macro>

<#--
}

/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
-->
<#macro macro_math_dsp_app_c_tasks_data>
</#macro>

<#--
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            bool appInitialized = true;
-->
<#macro macro_math_dsp_app_c_tasks_state_init>
</#macro>

<#--
            if (appInitialized)
            {
-->
<#macro macro_math_dsp_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_math_dsp_app_c_tasks_state_service_tasks>
            /* Run the state machine for servicing the MATH DSP task */
            ${APP_NAME?upper_case}_MATH_DSP_Task();

</#macro>

<#--
            break;
        }
-->
<#macro macro_math_dsp_app_c_tasks_states>
</#macro>

<#--
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application state machine. */
            break;
        }
    }
}
-->

<#macro macro_math_dsp_app_c_tasks_app_functions>
</#macro>
