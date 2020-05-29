/**
 * @file   (redacted)
 * @author Michael Wood
 *
 * Created on May 22, 2014, 00:08 AM
 */

//////////////////////////////////////////////////////////////////////
//
// Revision history:
//
// (redacted)
//
//////////////////////////////////////////////////////////////////////
#include (redacted)

#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)
#include (redacted)


//
// Task's private data
//
static unsigned char m_task_freq;           // task frequency (invocations/sec)
static unsigned int  m_year = DEFAULT_TIME_YEAR;            // 4-digit year
static unsigned int  m_month = DEFAULT_TIME_MONTH;           // two digit month (0~12)
static unsigned int  m_day = DEFAULT_TIME_DAY;             // day of year (0~365)
static unsigned int  m_monthDay = DEFAULT_TIME_MONTHDAY;        // day of month (0~31)
static unsigned char m_hour = 0;            // local standard time
static unsigned char m_minute = 0;
static unsigned char m_second = 0;
static unsigned char m_tick = 0;            // heartbeat counter
static (redacted)
static (redacted)
static unsigned char sec_split = 60;
static char m_btNewSecond = 0;
static char m_btNewMinute = 0;
static char m_btNewHour = 0;
static char m_btNewDay = 0;
static char m_btLeapYear = 0;
static unsigned char m_btThreeSecondFlag = 0;
static unsigned char m_btTenSecondFlag = 0;
static unsigned char m_btTenMinuteFlag = 0;
static long timeNow = 0;
static unsigned char m_bMorning = 1;				// Start assuming morning
static unsigned int m_trkFltDelay = 30;
static long m_aSunrise = 0;
static unsigned int m_aSunrisePos = 0;
static char m_bClockSet = 0;

const unsigned int monthDays[13] = {0,31,59,90,120,151,181,212,243,273,304,334,365};
const unsigned int leapyear_monthDays[13] = {0,31,60,91,121,152,182,213,244,274,305,335,366};

// for new clock test
static unsigned int m_lastsecond = 0;
static unsigned int m_thissecond = 0;
static unsigned int m_lastminute = 0;
static unsigned int m_thisminute = 0;
static unsigned char m_sec2 = 0;
static unsigned char m_sec2_one = 0;
static unsigned char m_sec2_ten = 0;
static unsigned char m_min2 = 0;
static unsigned char m_min2_one = 0;
static unsigned char m_min2_ten = 0;
static unsigned char m_hr2 = 0;
static unsigned int m_mday2 = DEFAULT_TIME_MONTHDAY;
static unsigned int m_day2 = DEFAULT_TIME_DAY;
static unsigned int m_mth2 = DEFAULT_TIME_MONTH;
static unsigned int m_yr2 = DEFAULT_TIME_YEAR;
char m_cLineOut[50];

// RTC (real time clock)
static char m_rtcEnabled = 1;

// RTCC (Real-Time Clock Circuit) values
static rtccTime rtcc_time, rtcc_time_now;
static rtccDate rtcc_date;

//
// Task's private functions
//

// Decimal / Binary Coded Decimal converters for RTCC
unsigned char BCDtoDEC (unsigned char bcd)
{
    unsigned char tens, ones;

    tens = ((bcd & 0xF0) >> 4);
    ones = (bcd & 0x0F);

    return (tens*10 + ones);
}


unsigned char DECtoBCD (unsigned char dec)
{
    unsigned char ret, tens, ones;

    ones = (unsigned char)((int)dec % 10);
    tens = (unsigned char)((int)dec / 10);

    ret = ((ones) | (tens << 4));

    return ret;
}

void IncrementDay (void)
{
    if (++m_day == 365)
    {
        //
        // Check if this is a leap year.
        //
        if ((m_year % 4 == 0 && m_year % 100 != 0)
             || m_year % 400 == 0)
            ;               // it's a leap year
        else
        {
            m_day = 0;
            m_year++;
        }
    }
    else if (m_day == 366)
    {
        m_day = 0;
        m_year++;
    }
}

char IsClockSet(void) { return m_bClockSet; }

void SetTime (unsigned char hr, unsigned char min, unsigned char sec)
{
    m_hour = hr;
    m_minute = min;
    m_second = sec;

    unsigned long time = (DECtoBCD(hr)<<24) & (DECtoBCD(min)<<16) & (DECtoBCD(sec)<<8);
    RtccWrEnable(1);
    RtccSetTime(time);
    while(RtccGetClkStat() != RTCC_CLK_ON)
        ;
#endif

}

void SetClock(int year, int month, int monthDay,
              unsigned char hr, unsigned char min, unsigned char sec)
{
    m_bClockSet = 1;

    // system.c contains the seconds-counter, and also keeps track of seconds
    // on the clock (0-59), so set this along with timekeeping.c variable
    SYSTEMSetSecond(sec);

    // TIMEKEEPINGTask() keeps track of the previous second, so when the current
    // second is new (incremented by system.c) it can begin the secondly
    // tasks. So set the two to be equal, to give us exactly 1 second before the
    // next secondly task set begins.
    m_lastsecond = sec;

    m_year = year;
    m_month = month;
    m_monthDay = monthDay;
    m_hour = hr;
    m_minute = min;
    m_second = sec;

    // Day of year calc (need to know if leap year)
    if ((m_year % 4 == 0 && m_year % 100 != 0) || m_year % 400 == 0)
    {
        m_btLeapYear = 1;   // Save as task variable
        m_day = leapyear_monthDays[m_month - 1] + m_monthDay;
    }

    else
    {
        m_btLeapYear = 0;
        m_day = monthDays[m_month - 1] + m_monthDay;
    }

    if (TIMEKEEPIsRtcSyncEnabled())
    {
        unsigned char CenturyBits = 0;  // 0 = 2000-2099, 1=2100-2199, 2=2200-2299
        unsigned char RtcRegisterBcdYear = TIMEKEEPRtcDECtoBCD((unsigned int)(m_year%100),RTC_ADDRESS_YEAR);
        unsigned char RtcRegisterBcdMonth = TIMEKEEPRtcDECtoBCD((unsigned int)m_month,RTC_ADDRESS_MONTH);
        unsigned char RtcRegisterBcdDayOfMonth = TIMEKEEPRtcDECtoBCD((unsigned int)m_monthDay,RTC_ADDRESS_DAY_OF_MONTH);
        unsigned char RtcRegisterBcdDayOfWeek = 0; // what to do with this?
        unsigned char RtcRegisterBcdCenturyAndHr = TIMEKEEPRtcDECtoBCD((unsigned int)m_hour,RTC_ADDRESS_CENTURY_AND_HR) + (CenturyBits<<4);
        unsigned char RtcRegisterBcdMin = TIMEKEEPRtcDECtoBCD((unsigned int)m_minute,RTC_ADDRESS_MIN);
        unsigned char RtcRegisterBcdSec = TIMEKEEPRtcDECtoBCD((unsigned int)m_second,RTC_ADDRESS_SEC);

        TIMEKEEPRtcWrite(RTC_ADDRESS_YEAR, (unsigned int)RtcRegisterBcdYear);
        TIMEKEEPRtcWrite(RTC_ADDRESS_MONTH, (unsigned int)RtcRegisterBcdMonth);
        TIMEKEEPRtcWrite(RTC_ADDRESS_DAY_OF_MONTH, (unsigned int)RtcRegisterBcdDayOfMonth);
        TIMEKEEPRtcWrite(RTC_ADDRESS_DAY_OF_WEEK, (unsigned int)RtcRegisterBcdDayOfWeek);
        TIMEKEEPRtcWrite(RTC_ADDRESS_CENTURY_AND_HR, (unsigned int)RtcRegisterBcdCenturyAndHr);
        TIMEKEEPRtcWrite(RTC_ADDRESS_MIN, (unsigned int)RtcRegisterBcdMin);
        TIMEKEEPRtcWrite(RTC_ADDRESS_SEC, (unsigned int)RtcRegisterBcdSec);
    }

}

/*
 * Update configs and timekeep.c's time and date with whatever is on RTC
 */
void SyncWithRTC(void)
{
    if (TIMEKEEPHasRTC())
    {
        m_bClockSet = 1;

        m_second = (unsigned char)TIMEKEEPRtcBCDtoDEC(TIMEKEEPRtcRead(RTC_ADDRESS_SEC),RTC_ADDRESS_SEC);

        SYSTEMSetSecond((unsigned int)m_second);

        // system.c contains the seconds-counter, and also keeps track of seconds
        // on the clock (0-59), so set this along with timekeeping.c variable
        SYSTEMSetSecond(m_second);

        // TIMEKEEPINGTask() keeps track of the previous second, so when the current
        // second is new (incremented by system.c) it can begin the secondly
        // tasks. So set the two to be equal, to give us exactly 1 second before the
        // next secondly task set begins.
        m_lastsecond = m_second;

        m_minute = (unsigned char)TIMEKEEPRtcBCDtoDEC(TIMEKEEPRtcRead(RTC_ADDRESS_MIN),RTC_ADDRESS_MIN);
        m_hour = (unsigned char)TIMEKEEPRtcBCDtoDEC(TIMEKEEPRtcRead(RTC_ADDRESS_CENTURY_AND_HR),RTC_ADDRESS_CENTURY_AND_HR);
        m_monthDay = (int)TIMEKEEPRtcBCDtoDEC(TIMEKEEPRtcRead(RTC_ADDRESS_DAY_OF_MONTH),RTC_ADDRESS_DAY_OF_MONTH);
        m_month = (int)TIMEKEEPRtcBCDtoDEC(TIMEKEEPRtcRead(RTC_ADDRESS_MONTH),RTC_ADDRESS_MONTH);
        m_year = (int)TIMEKEEPRtcBCDtoDEC(TIMEKEEPRtcRead(RTC_ADDRESS_YEAR),RTC_ADDRESS_YEAR) + 2000;


        // Leap year needed to calculate day-of-year
        if ((m_year % 4 == 0 && m_year % 100 != 0) || m_year % 400 == 0)
        {
            m_btLeapYear = 1;   // Save as task variable
            m_day = leapyear_monthDays[m_month - 1] + m_monthDay;
        }

        else
        {
            m_btLeapYear = 0;
            m_day = monthDays[m_month - 1] + m_monthDay;
        }

        // Sync azmeq and offsets
        TRACKSetAzimMotionEquation(TIMEKEEPFetchAzmeq());

        int azOff, elOff;

        TIMEKEEPFetchOffsets(&azOff, &elOff);
        TrkSetOffsets(azOff, elOff);
    }
}

/**
 * Does the hardware include the RTC?
 * @retrun rtc   1=yes, 0=no (int)
 */
int TIMEKEEPHasRTC (void)
{
    return atoi(CONTROLLER_ID)>110079;
}

void UpdateDate (rtccDate* pDt)
{
    // date
    m_year = 2000 + BCDtoDEC(pDt->year);

    if ((m_year % 4 == 0 && m_year % 100 != 0) || m_year % 400 == 0)
        m_btLeapYear = 1;
    else
        m_btLeapYear = 0;

    m_month = BCDtoDEC(pDt->mon);
    m_monthDay = BCDtoDEC(pDt->mday);
    if (m_btLeapYear)
        m_day = m_monthDay + leapyear_monthDays[m_month - 1];
    else
        m_day = m_monthDay + monthDays[m_month - 1];
}

void UpdateTime (rtccTime* pTm)
{
    // time
    unsigned char newMinute = BCDtoDEC(pTm->min);
    unsigned char newHour = BCDtoDEC(pTm->hour);

    if (m_minute != newMinute)
    {
        m_btNewMinute = 1;
        if (newHour != m_hour)
            m_btNewHour = 1;
        else
            m_btNewHour = 0;
    }
    else
        m_btNewMinute = 0;


    m_second = BCDtoDEC(pTm->sec);
    m_minute = newMinute;
    m_hour = newHour;
}

//
// Task's public functions
//

/**
 * Returns the enabled or disabled sync status of RTC
 * @return status  1=enabled, 0=disabled (int)
 */
unsigned int TIMEKEEPIsRtcSyncEnabled (void)
{
    if (TIMEKEEPHasRTC())
        return TIMEKEEPRtcRead(RTC_ADDRESS_ENABLED);
}

/**
 * Enable or disable RTC sync
 * @param status    1=enable, 0=disable
 */
void TIMEKEEPEnableRtcSync (unsigned int status)
{
    float azmeq = TRACKGetAzimMotionEquation();
    int azimOffset, elevOffset;

    TrkGetOffsets(&azimOffset, &elevOffset);

    // Enable/disable RTC
    TIMEKEEPRtcWrite(RTC_ADDRESS_ENABLED,status);

    // If turning on RTC sync..
    if (status == 1)
    {
        // ..immediately load RTC with current azmeq and offsets
        TIMEKEEPStoreAzmeq (azmeq);
        TIMEKEEPStoreOffsets(azimOffset, elevOffset);
    }
}


/**
 * Reads arbitrary data from user RAM location, which we use to as non-vol mem
 * for az motion equation
 * @return azmeq  Float value that is BCD-style decoded azmeq
 */
float TIMEKEEPFetchAzmeq (void)
{
    if (TIMEKEEPIsRtcSyncEnabled())
    {
        unsigned int integerPortion = TIMEKEEPRtcRead(RTC_ADDRESS_AZMEQ_INT);
        unsigned int decimalUpperPortion = TIMEKEEPRtcRead(RTC_ADDRESS_AZMEQ_DEC_HI);
        unsigned int decimalLowerPortion = TIMEKEEPRtcRead(RTC_ADDRESS_AZMEQ_DEC_LO);

        float ret;

        ret = (float)integerPortion + ((float)decimalUpperPortion/100) + ((float)decimalLowerPortion/10000);

        return ret;
    }
}

/**
 * Writes arbitrary data to user RAM location, which we use as non-vol mem
 * for az motion equation
 * @param azmeq    Float value to be BCD-style coded azmeq
 */
void TIMEKEEPStoreAzmeq (float azmeq)
{
    unsigned char integerPortion = azmeq;
    unsigned int ui2 = 10000*(azmeq - (float)integerPortion);

    unsigned char decimalUpperPortion = ui2/100;
    unsigned char decimalLowerPortion = (unsigned char)(ui2 - (unsigned int)decimalUpperPortion*100);

    if (TIMEKEEPIsRtcSyncEnabled())
    {
        TIMEKEEPRtcWrite(RTC_ADDRESS_AZMEQ_INT,(unsigned int)integerPortion);
        TIMEKEEPRtcWrite(RTC_ADDRESS_AZMEQ_DEC_HI,(unsigned int)decimalUpperPortion);
        TIMEKEEPRtcWrite(RTC_ADDRESS_AZMEQ_DEC_LO,(unsigned int)decimalLowerPortion);
    }
}

/**
 * Reads arbitrary data from user RAM location, which we use to as non-vol mem
 * for az and el solar offsets
 * @param   *pAzOff     Pointer to az offset (int)
 * @param   *pElOff     Pointer to el offset (int)
 */
void TIMEKEEPFetchOffsets (int *pAzOff, int *pElOff)
{
    if (TIMEKEEPIsRtcSyncEnabled())
    {
        // Read BCD-like representation from RTC memory
        int integerUpperPortion = (unsigned int)TIMEKEEPRtcRead(RTC_ADDRESS_AZOFF_HI);
        int integerLowerPortion = (unsigned int)TIMEKEEPRtcRead(RTC_ADDRESS_AZOFF_LO);

        // Decode BCD-like representation
        *pAzOff = integerUpperPortion * 100 + integerLowerPortion;

        // Read BCD-like representation from RTC memory
        integerUpperPortion = (unsigned int)TIMEKEEPRtcRead(RTC_ADDRESS_ELOFF_HI);
        integerLowerPortion = (unsigned int)TIMEKEEPRtcRead(RTC_ADDRESS_ELOFF_LO);

        // Decode BCD-like representation
        *pElOff = integerUpperPortion * 100 + integerLowerPortion;
    }

}

/**
 * Writes arbitrary data to user RAM location, which we use as non-vol mem
 * for az and el solar offsets
 * @param azOff     Az offset (int)
 * @param elOff     El offset (int)
 */
void TIMEKEEPStoreOffsets (int azOff, int elOff)
{
    if (TIMEKEEPIsRtcSyncEnabled())
    {
        // Build BCD-like representation of azimuth offset
        unsigned char integerUpperPortion = azOff/100;
        unsigned char integerLowerPortion = (unsigned char)(azOff - (unsigned int)integerUpperPortion*100);

        // Write to RTC memory
        TIMEKEEPRtcWrite(RTC_ADDRESS_AZOFF_HI,integerUpperPortion);
        TIMEKEEPRtcWrite(RTC_ADDRESS_AZOFF_LO,integerLowerPortion);

        // Build BCD-like representation of elevation offsets
        integerUpperPortion = elOff/100;
        integerLowerPortion = (unsigned char)(elOff - (unsigned int)integerUpperPortion*100);

        // Write to RTC memory
        TIMEKEEPRtcWrite(RTC_ADDRESS_ELOFF_HI,integerUpperPortion);
        TIMEKEEPRtcWrite(RTC_ADDRESS_ELOFF_LO,integerLowerPortion);
    }

}

/**
 * Decimal / Binary Coded Decimal converters for M41T93 RTC
 * @param   valBcd  Value as BCD
 * @param   address RTC address of the value (need to look up number of BCD bits)
 * @return          Value as DEC
 */
unsigned int TIMEKEEPRtcBCDtoDEC (unsigned int valBcd, char address)
{
    unsigned int tens, ones;
    unsigned int tensBitmask = 0;
    unsigned char bits = bcdBitsLookup[address];

    // Build ten's place bitmask based on the number of bits (right justified)
    while (bits--)  { tensBitmask += ((1<<4) << bits); }

    // Keep all of the ones
    ones = (valBcd & 0x000F);

    // Apply ten's place bitmask
    tens = ((valBcd & tensBitmask) >> 4);

    // Build DEC value and return
    return (tens*10 + ones);
}

/**
 * Decimal / Binary Coded Decimal converters for M41T93 RTC
 * @param   valDec  Value as dec
 * @param   address RTC address of the value (need to look up number of BCD bits)
 * @return          Value as BCD
 */
unsigned int TIMEKEEPRtcDECtoBCD (unsigned int valDec, char address)
{
    unsigned int tens, ones;
    unsigned int tensBitmask = 0;
    unsigned char bits = bcdBitsLookup[address];

    // Build ten's place bitmask based on the number of bits (right justified)
    while (bits--)  { tensBitmask += ((1<<4) << bits); }

    // How many ones
    ones = valDec % 10;

    // How many tens (and ensure no extra bits are set)
    tens = (((valDec / 10)) << 4) & tensBitmask;

    // Build BCD value and return
    return ones | tens;
}

/**
 * Read data from real time clock
 * @param   address Address to be read (one byte, value 0x01 through 0x1F)
 * @return  data    Data retrieved from real time clock in hex (one byte, any value)
 */
unsigned int TIMEKEEPRtcRead (unsigned int address)
{
    unsigned int data = 0;

    // Need to build a 16-bit message where:
    //      - the MSB (bit 7) is 0 (signals a write)
    //      - the following bits (bits 6-0) are the address
    unsigned int readTx=address<<8;

    data = SPI1Rx(readTx);



    // Data may come through padded with 1's in the first 8 bits
    if (data > 0xFF)
        data = data - 0xff00;

    return data;
}


/**
 * Write data to real time clock
 * @param   address Address to be written (one byte, value 0x01 through 0x1F)
 * @param   data    Data to be written (one byte, any value)
 */
void TIMEKEEPRtcWrite (unsigned int address, unsigned int data)
{
    unsigned int message = 0x8000 + (address<<8) + data;

    SPI1Tx(message);
}

/**
 * Initialize the real time clock
 */
void RtcInit (void)
{
    // To "Kickstart" the clock need to write a 1, then 0 to bit 7 of 0x01.
    // After writing the 0, would like 0x01 to retain whatever data it had
    // before we started this operation. So, read the data, make sure bit 7
    // is cleared, then send the data back to start the clock.

    unsigned int data = TIMEKEEPRtcRead(0x01);
    data &= 0x7F;                 // Bitmask ensures bit 7 is cleared

    // "Kickstart" clock
    TIMEKEEPRtcWrite(0x01,0x80);  // Set bit 7 of 0x01 to stop clock

    TIMEKEEPRtcWrite(0x01,data);  // Clear bit 7 of 0x01 to start clock

    TIMEKEEPRtcWrite(0x0C,0x00);  // Set halt bit (0x0C bit 6) to zero
                                  // (also sets Alarm 1 hour and RPT13 to zero)
}



/**
 * Initialize time-of-day task.
 * @param freq Frequency in Hz at which task will be invoked.
 */
void TIMEKEEPInit (unsigned char freq)
{
    m_task_freq = freq;
    m_tick = 0;
    m_bAccTrack = 0;
    m_bNoNight = 0;
    ER_az_ct = 0;
    ER_az_retries = 0;
    ER_el_ct = 0;
    ER_el_retries = 0;
    ER_stow_ct = 0;
    ER_stow_retries = 0;
    TRACKCalcSPASunRiseSetNoon (m_year, m_month, m_monthDay);

    if (TIMEKEEPHasRTC())
    {
        SPI1Init();
        RtcInit();
        if (TIMEKEEPIsRtcSyncEnabled())
            SyncWithRTC();
    }
}

/**
 * Utility to convert time in seconds since midnight to hour, minute, second.
 *
 * @param secs  Seconds since midnight.
 * @param pHr   Pointer to address to receive hour.
 * @param pMin  Pointer to address to receive minute.
 * @param pSec  Pointer to address to receive second.
 */
void ConvertSecsToHrMinSec (long secs, unsigned char *pHr,
                            unsigned char *pMin,
                            unsigned char *pSec)
{
    long hs, ms;
    *pHr = (unsigned char) (secs / 3600);
    hs = *pHr * 3600;
    *pMin = (unsigned char) ((secs - hs) / 60);
    ms = *pMin * 60;
    *pSec = (unsigned char) (secs - hs - ms);
}

/**
 * Set year, day of year, and current local standard time.
 *
 * @param year  Year (4 digits).
 * @param day   Day of year (0-365).
 * @param hr    Hour (0-23).
 * @param min   Minute (0-59).
 * @param sec   Second (0-59).
 */
void SetDateAndTime (unsigned int year, unsigned int day,
                     unsigned char hr, unsigned char min,
                     unsigned char sec)
{

    m_year = year;
    m_day = day;
    //SetTime (hr, min, sec);

    if ((m_year % 4 == 0 && m_year % 100 != 0) || m_year % 400 == 0)
        m_btLeapYear = 1;
    else
        m_btLeapYear = 0;

    int p=1;
    if(m_btLeapYear)
    {
        for (p=1; p < 13; p++)
        {
            if(day <= leapyear_monthDays[p])
            {
                m_month = p;
                m_monthDay = (unsigned int)(day - leapyear_monthDays[m_month-1]);
                break;
            }
        }
    }
    else
    {
        for (p=1; p < 13; p++)
        {
            if(day <= monthDays[p])
            {
                m_month = p;
                m_monthDay = (unsigned int)(day - monthDays[m_month-1]);
                break;
            }
        }
    }

    SetClock(m_year, m_month, m_monthDay, hr, min, sec);
    TRACKCalcSPASunRiseSetNoon (m_year, m_month, m_monthDay);
}

/**
 * Set year, day of year, and current local standard time.
 * Same as above, but input is in UTC.
 *
 * @param year  Year (4 digits).
 * @param day   Day of year (0-365).
 * @param hr    Hour (0-23).
 * @param min   Minute (0-59).
 * @param sec   Second (0-59).
 */
void SetDateAndTimeUTC (unsigned int year, unsigned int day,
                       unsigned int hr, unsigned int min,
                       unsigned int sec)
{
    signed int offset = TRACKGetTimezone();
    signed int hour_test = (hr + offset);
    signed int day_corrected = day;
    if (hour_test < 0)  // if the UTC offset puts us back past midnight
    {
        if (--day_corrected < 0)  // go back a day; if that wraps to a new year
        {
            if (year > 0)
            {
                year--;  // wind back one year
                if ((year % 4 == 0 && year % 100 != 0)
                        || year % 400 == 0)  // if it's a leap year
                {
                    day = 365;  // last day of leap year (0-indexed)
                }
                else
                {
                    day = 364;  // last day of normal year (0-indexed)
                }
            }
        }
        else
            day = day_corrected;
    }

    m_year = year;
    m_day = day;
    unsigned char corrected_hr;
    if (hour_test < 0)
        corrected_hr = (unsigned char)(24 + hour_test);
    else
        corrected_hr = (unsigned char)(hour_test);

    //SetTime (corrected_hr, min, sec);

    if ((m_year % 4 == 0 && m_year % 100 != 0) || m_year % 400 == 0)
        m_btLeapYear = 1;
    else
        m_btLeapYear = 0;

    int p=1;
    if(m_btLeapYear)
    {
        for (p=1; p < 13; p++)
        {
            if(day <= leapyear_monthDays[p])
            {
                m_month = p;
                m_monthDay = (unsigned int)(day - leapyear_monthDays[m_month-1]);
                break;
            }
        }
    }
    else
    {
        for (p=1; p < 13; p++)
        {
            if(day <= monthDays[p])
            {
                m_month = p;
                m_monthDay = (unsigned int)(day - monthDays[m_month-1]);
                break;
            }
        }
    }

    if (m_btLeapYear)
        m_monthDay = (unsigned int)(day - leapyear_monthDays[m_month-1]);
    else
        m_monthDay = (unsigned int)(day - monthDays[m_month-1]);

    SetClock(m_year, m_month, m_monthDay, corrected_hr, min, sec);
    TRACKCalcSPASunRiseSetNoon (m_year, m_month, m_monthDay);
}


/**
 * Set year, day of year, and current local standard time from SPP
 *
 * @param year  Year (4 digits).
 * @param month Month (2 digits).
 * @param day   Day of month (1-31).
 * @param hr    Hour (0-23).
 * @param min   Minute (0-59).
 * @param sec   Second (0-59).
 */
void SetDateAndTime_SSP (unsigned int year, unsigned int month, unsigned int day,
                     unsigned char hr, unsigned char min,
                     unsigned char sec)
{

    m_year = year;
    m_month = month;
    m_monthDay = day;
    SetTime (hr, min, sec);

    if ((m_year % 4 == 0 && m_year % 100 != 0) || m_year % 400 == 0)
        m_btLeapYear = 1;
    else
        m_btLeapYear = 0;

    if (m_btLeapYear)
        m_day = (unsigned int)(day + leapyear_monthDays[m_month-1]);
    else
        m_day = (unsigned int)(day + monthDays[m_month-1]);

    TRACKCalcSPASunRiseSetNoon (m_year, m_month, m_monthDay);
}

/**
 * Set year, day of year, and current local standard time.
 * Same as above, but input is in UTC.
 *
 * @param year  Year (4 digits).
 * @param day   Day of year (0-365).
 * @param hr    Hour (0-23).
 * @param min   Minute (0-59).
 * @param sec   Second (0-59).
 */
void SetDateAndTimeUTC_SSP (unsigned int year, unsigned int month, unsigned int monthDay,
                       unsigned int hr, unsigned int min,
                       unsigned int sec)
{
    year += 2000;
    signed int offset = TRACKGetTimezone();
    signed int hour_test = (hr + offset);
    signed int monthDay_corrected = monthDay;
    signed int month_corrected = month;
    unsigned int day;
    if (hour_test < 0)  // if the UTC offset puts us back past midnight
    {
        if (--monthDay_corrected < 0)  // go back a day; if that wraps to a new month
        {
            if (--month_corrected < 1)
            {
                month_corrected = 12;
                if(year > 0)
                    year--;  // wind back one year
                if ((year % 4 == 0 && year % 100 != 0)
                        || year % 400 == 0)  // if it's a leap year
                {
                    m_btLeapYear = 1;
                    day = monthDay_corrected + leapyear_monthDays[month-1];  // last day of leap year (0-indexed)
                    monthDay = monthDay_corrected;
                }
                else
                {
                    m_btLeapYear = 0;
                    day = monthDay_corrected + monthDays[month-1];  // last day of normal year (0-indexed)
                    monthDay = monthDay_corrected;
                }
            }
            else // month doesn't wrap
            {
                month = month_corrected;
                if ((year % 4 == 0 && year % 100 != 0)
                        || year % 400 == 0)  // if it's a leap year
                {
                    m_btLeapYear = 1;
                    day = monthDay_corrected + leapyear_monthDays[month];  // last day of leap year (0-indexed)
                    monthDay = monthDay_corrected;
                }
                else
                {
                    m_btLeapYear = 0;
                    day = monthDay_corrected + monthDays[month];  // last day of normal year (0-indexed)
                    monthDay = monthDay_corrected;
                }
            }
        }
        else
        {
            if ((year % 4 == 0 && year % 100 != 0)
                    || year % 400 == 0)  // if it's a leap year
            {
                m_btLeapYear = 1;
                day = monthDay_corrected + leapyear_monthDays[month];  // last day of leap year (0-indexed)
                monthDay = monthDay_corrected;
            }
            else
            {
                m_btLeapYear = 0;
                day = monthDay_corrected + monthDays[month];  // last day of normal year (0-indexed)
                monthDay = monthDay_corrected;
            }
        }
    }
    else
    {
        if ((year % 4 == 0 && year % 100 != 0)
                || year % 400 == 0)  // if it's a leap year
        {
            m_btLeapYear = 1;
            day = monthDay_corrected + leapyear_monthDays[month];  // last day of leap year (0-indexed)
            monthDay = monthDay_corrected;
        }
        else
        {
            m_btLeapYear = 0;
            day = monthDay_corrected + monthDays[month];  // last day of normal year (0-indexed)
            monthDay = monthDay_corrected;
        }
    }

    m_year = year;
    m_month = month;
    m_monthDay = monthDay;
    m_day = day;
    unsigned char corrected_hr;
    if (hour_test < 0)
        corrected_hr = (unsigned char)(24 + hour_test);
    else
        corrected_hr = (unsigned char)(hour_test);
    SetTime (corrected_hr, min, sec);

    TRACKCalcSPASunRiseSetNoon (m_year, m_month, m_monthDay);
}


/**
 * Get day and year.
 *
 * @param pYr   Address to receive year.
 * @param pDay  Address to receive day.
 */
void GetDate (unsigned int *pYr, unsigned int *pDay)
{
    *pYr = m_year;
    *pDay = m_day;
}

//
// Date accessors
//
unsigned int GetYear    (void) { return m_year;         }
unsigned int GetMonth   (void) { return m_month;        }
unsigned int GetMonthDay(void) { return m_monthDay;     }

/**
 * Get current local standard time.
 *
 * @param pHr   Address to receive hour.
 * @param pMin  Address to receive minute.
 * @param pSec  Address to receive second.
 */
void GetTime (unsigned char *pHr, unsigned char *pMin, unsigned char *pSec)
{

    *pHr = m_hour;
    *pMin = m_minute;
    *pSec = m_second;
}

unsigned char IsDay (void)
{
    unsigned char d = 0;
    if ((TrkCmpSunrise (m_hour, m_minute, m_second) >= 0) &&
            (TrkCmpSunset (m_hour, m_minute, m_second) < 0))
    {
        d = 1;
    }
    return d;
}

/**
 * Sets or clears accelerated tracking condition.
 * @param bSet 0 => normal tracking; 1 => accelerated tracking.
 */
void SetAccTrack (unsigned char bSet)
{
    if(bSet)
    {
        if(bSet == 2)
            sec_split = 10;
        else
            sec_split = 60;

        SYSTEMSetTickMod(10);
        TrkSetAzimThreshold(100);
    }
    else
    {
        sec_split = 60;
        SYSTEMSetTickMod(100);
        TrkSetAzimThreshold(15);
    }

    m_bAccTrack = bSet;
}

void SetNoNight (unsigned char bSet)
{
    m_bNoNight = bSet;
}

//////////////////////////////////////////////////////////////////////
//
//  void TIMESetTrkFltDelay (unsigned int delay)
//
//  Set the number of minutes to wait before trying to track if, during
//  the day you stop tracking while tracking is enabled
//
//  Arguments
//	    delay:       number of minutes
//
//////////////////////////////////////////////////////////////////////
void TIMESetTrkFltDelay (unsigned int delay)
{
    m_trkFltDelay = delay;
}

//////////////////////////////////////////////////////////////////////
//
//  unsigned int TIMEGetTrkFltDelay (void)
//
//  Get the number of minutes to wait before trying to track if, during
//  the day you stop tracking while tracking is enabled
//
//  returns
//	    number of minutes
//
//////////////////////////////////////////////////////////////////////
unsigned int TIMEGetTrkFltDelay (void)
{
    return m_trkFltDelay;
}

/**
 * Check for accelerated tracking condition.
 * @return 0 => normal tracking; 1 => accelerated tracking.
 */
unsigned char IsAccTrack (void)
{
    return m_bAccTrack;
}

unsigned char IsNoNight (void)
{
    return m_bNoNight;
}

unsigned char IsMorning (void)          { return m_bMorning;    }

unsigned char ThreeSecondFlag (void)    { return m_btThreeSecondFlag; }
unsigned char TenSecondFlag (void)      { return m_btTenSecondFlag; }
unsigned char TenMinuteFlag (void)      { return m_btTenMinuteFlag; }

//////////////////////////////////////////////////////////////////////
//
// Time-of-day task loop, which is called every heartbeat
//
//////////////////////////////////////////////////////////////////////
/**
 * Time-of-day task loop, which is called every heartbeat.
 * Updates what time it is every second, and every 10 seconds, it checks if
 * it should be tracking, and if so, it tracks. Under certain circumstances it
 * submits state change requests.
 */
void TIMEKEEPTask (void)
{

    m_btThreeSecondFlag = 0;
    m_btTenSecondFlag = 0;
    m_btTenMinuteFlag = 0;

#ifndef NO_RTCC
    if(!IsAccTrack()) // use RTCC
    {
        // see if this is a new second
        m_thissecond = BCDtoDEC(rtcc_time.sec);
        if (m_thissecond != m_lastsecond)
            m_btNewSecond = 1;
        m_lastsecond = m_thissecond;

        // see if this is a new minute
        m_thisminute = BCDtoDEC(rtcc_time.min);
        if (m_thisminute != m_lastminute)
            m_btNewMinute = 1;
        m_lastminute = m_thisminute;
    }
    else // use interrupt time
    {
        m_thissecond = SYSTEMSecond();
        if (m_thissecond != m_lastsecond)
        {
            m_btNewSecond = 1;
            m_second = m_thissecond%60;
            if (m_second%sec_split == 0)
            {
                m_second = 0;
                m_btNewMinute = 1;
                if (++m_minute%60 == 0)
                {
                    m_minute = 0;
                    if (++m_hour%24 == 0)
                    {
                        m_hour = 0;
                    }
                }
            }
        }
        m_lastsecond = m_thissecond;
    }
#endif

    ++m_tick;

    m_thissecond = SYSTEMSecond();
    if (m_thissecond != m_lastsecond)
    {
        m_btNewSecond = 1;
        m_second = m_thissecond%60;
        if (m_second%sec_split == 0)
        {
            m_second = 0;
            m_btNewMinute = 1;
            if (++m_minute%60 == 0)
            {
                m_minute = 0;
                if (++m_hour%24 == 0)
                {
                    m_hour = 0;
                    m_btNewDay = 1;

                    if (++m_day == 365)
                    {
                        //
                        // Check if this is a leap year.
                        //
                        if ((m_year % 4 == 0 && m_year % 100 != 0)
                             || m_year % 400 == 0)
                            ;               // it's a leap year
                        else
                        {
                            m_day = 0;
                            m_year++;
                        }
                    }
                    else if (m_day == 366)
                    {
                        m_day = 0;
                        m_year++;
                    }
                }
            }
        }
    }
    m_lastsecond = m_thissecond;


    //if (m_tick == m_task_freq)              // has 1 sec elapsed?
    if (m_btNewSecond)
    {
        m_tick = 0;

        if (m_second == 0 && m_minute == 0)
        {
            if (TIMEKEEPIsRtcSyncEnabled())
                SyncWithRTC();
            else
                STATUSSendReq("TIME\n");

            if (m_hour == 0)
            {
                STATUSSendReq ("CONFIGS\n");
                if (CUSTOMIsNightAzimHome())
                    AzimHomeLost();
                if (CUSTOMIsNightElevHome())
                    ElevHomeLost();
                TRACKCalcSPASunRiseSetNoon (m_year, m_month, m_monthDay);
            }
        }


        //
        // Compare time to solarnoon, declare morning/afternoon
        //
        timeNow = 3600 * (long)m_second +
                60 * (long)m_minute +
                (long)m_hour;


        if (timeNow < TrkGetSolarNoon())
                m_bMorning = 1;
        else
                m_bMorning = 0;

        //
        // Other Task Scheduling
        //

        // Anemo Task
        if (m_second % 3 == 0)
            m_btThreeSecondFlag = 1;


        //
        // Check every 10 seconds to determine tracking position.
        //
        if (m_second % 10 == 0 || IsAccTrack())
        {
            unsigned char bSunrise = 0;
            unsigned int aSunrisePos = 0;
            unsigned int eSunrisePos = 0;

            m_btTenSecondFlag = 1;



            //
            // If it's day
            //
            if ((TrkCmpSunrise (m_hour, m_minute, m_second) >= 0) &&
                    (TrkCmpSunset (m_hour, m_minute, m_second) < 0))
            {
                switch (tracker_state)
                {
                    case A_Tracking:
                        if (!ADCEStopIsIn() && CUSTOMVoltageOK())
                            TrkToPos (m_hour, m_minute, m_second);
                        break;

                    case A_Waiting:
                        SubmitStateRequest (Tracking_req);
                        break;
                }
            }
            else    // if it's night
            {
                if (IsNoNight())           // for accelerated tracking ...
                {
                    unsigned char hh, mm, ss;
                    long sunrise = TrkGetSunrise();
                    ConvertSecsToHrMinSec (sunrise, &hh, &mm, &ss);
                    SetTime (hh, mm, ss);
                    //TrkStartTracking();
                }
                switch (tracker_state)
                {
                    case A_Waiting:
                        if (CUSTOMIsNightActive() && (TrkCmpSunset(m_hour, m_minute, m_second) > 0))  // active night and before midnight
                        {
                            if (!(CUSTOMGetAzimNightPos() == 0 || CUSTOMGetElevNightPos() == 0))
                            {
                                if (!AzimIsMoving() &&
                                       !ElevIsMoving() &&
                                       ((abs (AzimGetPos() - CUSTOMGetAzimNightPos())) > 100))
                                {
                                    if (!ADCEStopIsIn() && CUSTOMVoltageOK())
                                        AzimMoveAbs(CUSTOMGetAzimNightPos(), AzimGetTrkSpd());
                                }
                                if (!ElevIsMoving() &&
                                        !AzimIsMoving() &&
                                        ((abs (ElevGetPos() - CUSTOMGetElevNightPos())) > ElevGetCountsPerDegree()))
                                {
                                    if (!ADCEStopIsIn() && CUSTOMVoltageOK())
                                        ElevMoveAbs(CUSTOMGetElevNightPos(), ElevGetTrackDownSpeedVal());
                                }
                            }
                        }
                        else  // if it's after midnight but before morning
                        {
                            if (!AzimIsMoving() &&
                                    !ElevIsMoving() &&
                                    ((abs (AzimGetPos() - TrkGetAzimSunrisePos())) > 100))
                            {
                                if (!ADCEStopIsIn() && CUSTOMVoltageOK())
                                    TrkReturnToSunrise();
                            }
                            if (CUSTOMIsNightStowed())
                            {
                                if (!ElevIsMoving() &&
                                        !AzimIsMoving() &&
                                        !ElevIsStowed())
                                {
                                    if (!ADCEStopIsIn() && CUSTOMVoltageOK())
                                        ElevStow();
                                }
                            }
                            else
                            {
                                if (!ElevIsMoving() &&
                                        !AzimIsMoving() &&
                                        ((abs (ElevGetPos() - ElevVerticalPosition())) > ElevGetCountsPerDegree()))
                                {
                                    if (!ADCEStopIsIn() && CUSTOMVoltageOK())
                                        ElevVertical();
                                }
                            }
                        }
                        break;

                    case A_Tracking:
                        SubmitStateRequest (Waiting_req);
                        break;

                }
            }

            if ((boot_loop_ct <= (2 * BOOT_LOOP_MAX)) && (m_second % 10 == 0))
            {
                boot_loop_ct++;     // increment boot loop count every 10 seconds

                if((boot_loop_ct == 18)&!TIMEKEEPIsRtcSyncEnabled())  // 3 min (lets BBB boot up)
                {
                    STATUSSendReq("TIME\n");
                    STATUSSendReq("CONFIGS\n");
                }
            }
        }


        //
        // Every 5 minutes, if you're in an Error Recovery state, increment
        // the error recovery timer.
        //
        if (((m_minute%5) == 0) && (m_btNewMinute))    // every 5 minutes
        {
            if (STATEIsFixingError())
            {
                // if error counter times out and there are no motor-related
                // errors, then we can stop trying to fix errors
                if (STATEIsFixingAz())
                {
                    if (ER_az_ct++ > ER_AZ_CT_MAX && !(ErrorIsAzStall()))
                    {
                        STATEClearFixAzFlag();
                        TRACKResetAzSpeed();
                    }
                }
                if (STATEIsFixingEl())
                {
                    if (ER_el_ct++ > ER_EL_CT_MAX && !(ErrorIsElStall()))
                    {
                        STATEClearFixElFlag();
                        TRACKResetElSpeed();
                    }
                }
                if (STATEIsFixingStow())
                {
                    if (ER_stow_ct++ > ER_STOW_CT_MAX && !(ErrorIsStowStall()))
                    {
                        STATEClearFixStowFlag();
                    }
                }
            }
        }

        if ((m_btNewMinute) && ((m_minute % 10) == 0))
        {
            m_btTenMinuteFlag = 1;
            if(!IsClockSet())
                STATUSSendGatewayAlert(GWA_RESET);
        }

        // the new second/minute is over
        m_btNewSecond = 0;
        m_btNewMinute = 0;
    }
}
