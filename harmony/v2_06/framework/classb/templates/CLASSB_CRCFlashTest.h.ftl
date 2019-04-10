/*******************************************************************************
  Function:
    unsigned int CLASSB_CRCFlashTest(char* startAddress, char* endAddress, unsigned int crcPoly, unsigned int crcSeed)

  Summary:
    The Flash CRC16 test implements the periodic modified checksum
    H.2.19.3.1 as defined by the IEC 60730 standard.

  Description:
    This routine  detects the single bit Faults in the invariable memory.
    The invariable memory in a system, such as Flash and EEPROM memory,
    contains data that is not intended to vary during the program execution.

    The test calculates the 16 bit CRC of the supplied memory area
    using the standard LFSR (Linear Feedback Shift Register) implementation.
    It calculates over the memory area between the startAddress and endAddress and returns the CRC Value.
    The 16 bit CRC is calculated using the supplied generator polynomial and initial seed.
    Different generator polynomials can be used as indicated above.
                                                                             
  Precondition:
    None.

  Parameters:
    startAddress    - start Address of the memory area to start CRC calculation from

    endAddress      - final address for which the CRC is calculated

    crcPoly         - the generator polynomial to be used.
                      One of the standard supplied polynomials can be used
                      as well as other user defined ones.

    crcSeed         - the initial value in the CRC LFSR.
                      The usual recommended value is 0xffff.
                          
  Returns:
    The value of the calculated CRC over the specified memory area.

  Example:
    <code>
    unsigned int crcRes=CLASSB_CRCFlashTest(startAddress, endAddress, CRC_16_GEN_POLY, 0xffff);
    if(crcRes==prevCalculatedCrc)
    {
        // process test success
    }
    else
    {
        // process tests failure: the CRC of the memory area has changed.
    }
    </code>

  Remarks:
    This is a non-destructive memory test.
    
    The startAddress and endAdress over which the CRC value is calculated are
    PIC32 variant and application dependent. They are run-time parameters.
    
  *****************************************************************************/
unsigned int CLASSB_CRCFlashTest(char* startAddress, char* endAddress, unsigned int crcPoly, unsigned int crcSeed);
