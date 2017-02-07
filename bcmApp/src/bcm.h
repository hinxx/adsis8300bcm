/* bcm.h
 *
 * This is a driver for a BCM based on Struck SIS8300 digitizer.
 *
 * Author: Hinko Kocevar
 *         ESS ERIC, Lund, Sweden
 *
 * Created:  January 22, 2017
 *
 */

#include <stdint.h>
#include <epicsEvent.h>
#include <epicsTime.h>
#include <asynNDArrayDriver.h>

#include <sis8300drv.h>

#define BCM_NUM_CHANNELS      10

#define BCM_IRQ_WAIT_TIME     0

#define BcmAllAlarmsString			"BCM_ALL_ALARMS"
#define BcmAuxClkAlarmString			"BCM_AUX_CLK_ALARM"
#define BcmCalPulseString			"BCM_CAL_PULSE"
#define BcmClkFreqString			"BCM_CLK_FREQ"
#define BcmClkTrigAlarmsFirstString			"BCM_CLK_TRIG_ALARMS_FIRST"
#define BcmClkTrigAlarmsLatchedString			"BCM_CLK_TRIG_ALARMS_LATCHED"
#define BcmDiffWarnResetString			"BCM_DIFF_WARN_RESET"
#define BcmFlatTopTimeString			"BCM_FLAT_TOP_TIME"
#define BcmFwVersionString			"BCM_FW_VERSION"
#define BcmMainClkAlarmString			"BCM_MAIN_CLK_ALARM"
#define BcmMaxPulseWidthString			"BCM_MAX_PULSE_WIDTH"
#define BcmMinTrigPeriodString			"BCM_MIN_TRIG_PERIOD"
#define BcmResetAlarmsString			"BCM_RESET_ALARMS"
#define BcmRfqTransparencyString			"BCM_RFQ_TRANSPARENCY"
#define BcmStatusString			"BCM_STATUS"
#define BcmSwVersionString			"BCM_SW_VERSION"
#define BcmTrigPeriodString			"BCM_TRIG_PERIOD"
#define BcmTrigPeriodSrcSelString			"BCM_TRIG_PERIOD_SRC_SEL"
#define BcmTrigTooFastAlarmString			"BCM_TRIG_TOO_FAST_ALARM"
#define BcmTrigTooLongAlarmString			"BCM_TRIG_TOO_LONG_ALARM"
#define BcmTrigTooShortAlarmString			"BCM_TRIG_TOO_SHORT_ALARM"
#define BcmTrigWidthString			"BCM_TRIG_WIDTH"

#define BcmAdcAlarmsFirstString			"BCM_ADC_ALARMS_FIRST"
#define BcmAdcAlarmsLatchedString			"BCM_ADC_ALARMS_LATCHED"
#define BcmAdcOffsetString			"BCM_ADC_OFFSET"
#define BcmAdcOffsErrorAvgString			"BCM_ADC_OFFS_ERROR_AVG"
#define BcmAdcOffsErrorIntegString			"BCM_ADC_OFFS_ERROR_INTEG"
#define BcmAdcScaleString			"BCM_ADC_SCALE"
#define BcmCalPulseAmplEarlyString			"BCM_CAL_PULSE_AMPL_EARLY"
#define BcmCalPulseAmplLateString			"BCM_CAL_PULSE_AMPL_LATE"
#define BcmDrooprateString			"BCM_DROOPRATE"
#define BcmDroopBaselineString			"BCM_DROOP_BASELINE"
#define BcmDroopErrorString			"BCM_DROOP_ERROR"
#define BcmErrantThresholdString			"BCM_ERRANT_THRESHOLD"
#define BcmErrantThresholdAlarmString			"BCM_ERRANT_THRESHOLD_ALARM"
#define BcmFlatTopChargeString			"BCM_FLAT_TOP_CHARGE"
#define BcmLocationMebtString			"BCM_LOCATION_MEBT"
#define BcmLocationRfqString			"BCM_LOCATION_RFQ"
#define BcmLowerThresholdString			"BCM_LOWER_THRESHOLD"
#define BcmLowerThresholdAlarmString			"BCM_LOWER_THRESHOLD_ALARM"
#define BcmNoiseFilterString			"BCM_NOISE_FILTER"
#define BcmOverflowAlarmString			"BCM_OVERFLOW_ALARM"
#define BcmPulseChargeString			"BCM_PULSE_CHARGE"
#define BcmPulsePastLimitAlarmString			"BCM_PULSE_PAST_LIMIT_ALARM"
#define BcmPulsePastTriggerAlarmString			"BCM_PULSE_PAST_TRIGGER_ALARM"
#define BcmPulseWidthString			"BCM_PULSE_WIDTH"
#define BcmStuckAlarmString			"BCM_STUCK_ALARM"
#define BcmTrigFineDelayString			"BCM_TRIG_FINE_DELAY"
#define BcmTrigToPulseString			"BCM_TRIG_TO_PULSE"
#define BcmUpperThresholdString			"BCM_UPPER_THRESHOLD"
#define BcmUpperThresholdAlarmString			"BCM_UPPER_THRESHOLD_ALARM"

#define BcmProbeSourceSelectString			"BCM_PROBE_SOURCE_SELECT"

class epicsShareClass Bcm : public ADSIS8300 {
public:
	Bcm(const char *portName, const char *devicePath,
			int maxAddr, int numTimePoints, NDDataType_t dataType,
			int maxBuffers, size_t maxMemory, int priority, int stackSize);
	~Bcm();

    /* These are the methods that we override from asynNDArrayDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual void report(FILE *fp, int details);

protected:
	int mBcmAllAlarms;
	#define FIRST_BCM_PARAM mBcmAllAlarms
	int mBcmAuxClkAlarm;
	int mBcmCalPulse;
	int mBcmClkFreq;
	int mBcmClkTrigAlarmsFirst;
	int mBcmClkTrigAlarmsLatched;
	int mBcmDiffWarnReset;
	int mBcmFlatTopTime;
	int mBcmFwVersion;
	int mBcmMainClkAlarm;
	int mBcmMaxPulseWidth;
	int mBcmMinTrigPeriod;
	int mBcmResetAlarms;
	int mBcmRfqTransparency;
	int mBcmStatus;
	int mBcmSwVersion;
	int mBcmTrigPeriod;
	int mBcmTrigPeriodSrcSel;
	int mBcmTrigTooFastAlarm;
	int mBcmTrigTooLongAlarm;
	int mBcmTrigTooShortAlarm;
	int mBcmTrigWidth;

	int mBcmAdcAlarmsFirst;
	int mBcmAdcAlarmsLatched;
	int mBcmAdcOffset;
	int mBcmAdcOffsErrorAvg;
	int mBcmAdcOffsErrorInteg;
	int mBcmAdcScale;
	int mBcmCalPulseAmplEarly;
	int mBcmCalPulseAmplLate;
	int mBcmDrooprate;
	int mBcmDroopBaseline;
	int mBcmDroopError;
	int mBcmErrantThreshold;
	int mBcmErrantThresholdAlarm;
	int mBcmFlatTopCharge;
	int mBcmLocationMebt;
	int mBcmLocationRfq;
	int mBcmLowerThreshold;
	int mBcmLowerThresholdAlarm;
	int mBcmNoiseFilter;
	int mBcmOverflowAlarm;
	int mBcmPulseCharge;
	int mBcmPulsePastLimitAlarm;
	int mBcmPulsePastTriggerAlarm;
	int mBcmPulseWidth;
	int mBcmStuckAlarm;
	int mBcmTrigFineDelay;
	int mBcmTrigToPulse;
	int mBcmUpperThreshold;
	int mBcmUpperThresholdAlarm;

	int mBcmProbeSourceSelect;
	#define LAST_BCM_PARAM mBcmProbeSourceSelect

private:

};

#define NUM_BCM_PARAMS ((int)(&LAST_BCM_PARAM - &FIRST_BCM_PARAM + 1))
