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

/* XXX: Sort these registers by address */
#define BCM_ALL_ALARMS_REG			0x431
#define BCM_AUX_CLK_ALARM_REG			0x432
#define BCM_CAL_PULSE_REG			0x414
#define BCM_CLK_FREQ_REG			0x421
#define BCM_CLK_TRIG_ALARMS_FIRST_REG			0x434
#define BCM_CLK_TRIG_ALARMS_LATCHED_REG			0x433
#define BCM_DIFF_WARN_RESET_REG			0x411
#define BCM_FLAT_TOP_TIME_REG			0x424
#define BCM_FW_VERSION_REG			0x400
#define BCM_MAIN_CLK_ALARM_REG			0x432
#define BCM_MAX_PULSE_WIDTH_REG			0x413
#define BCM_MIN_TRIG_PERIOD_REG			0x412
#define BCM_RESET_ALARMS_REG			0x430
#define BCM_RFQ_TRANSPARENCY_REG			0x410
#define BCM_STATUS_REG			0x420
#define BCM_SW_VERSION_REG			0x401
#define BCM_TRIG_PERIOD_REG			0x422
#define BCM_TRIG_PERIOD_SRC_SEL_REG			0x402
#define BCM_TRIG_TOO_FAST_ALARM_REG			0x432
#define BCM_TRIG_TOO_LONG_ALARM_REG			0x432
#define BCM_TRIG_TOO_SHORT_ALARM_REG			0x432
#define BCM_TRIG_WIDTH_REG			0x423

#define BCM_ADC_ALARMS_FIRST_REG			0x1A
#define BCM_ADC_ALARMS_LATCHED_REG			0x19
#define BCM_ADC_OFFSET_REG			0x05
#define BCM_ADC_OFFS_ERROR_AVG_REG			0x10
#define BCM_ADC_OFFS_ERROR_INTEG_REG			0x11
#define BCM_ADC_SCALE_REG			0x01
#define BCM_CAL_PULSE_AMPL_EARLY_REG			0x17
#define BCM_CAL_PULSE_AMPL_LATE_REG			0x18
#define BCM_DROOPRATE_REG			0x06
#define BCM_DROOP_BASELINE_REG			0x07
#define BCM_DROOP_ERROR_REG			0x13
#define BCM_ERRANT_THRESHOLD_REG			0x04
#define BCM_ERRANT_THRESHOLD_ALARM_REG			0x08
#define BCM_FLAT_TOP_CHARGE_REG			0x16
#define BCM_LOCATION_MEBT_REG			0x07
#define BCM_LOCATION_RFQ_REG			0x07
#define BCM_LOWER_THRESHOLD_REG			0x03
#define BCM_LOWER_THRESHOLD_ALARM_REG			0x08
#define BCM_NOISE_FILTER_REG			0x07
#define BCM_OVERFLOW_ALARM_REG			0x08
#define BCM_PULSE_CHARGE_REG			0x15
#define BCM_PULSE_PAST_LIMIT_ALARM_REG			0x08
#define BCM_PULSE_PAST_TRIGGER_ALARM_REG			0x08
#define BCM_PULSE_WIDTH_REG			0x14
#define BCM_STUCK_ALARM_REG			0x08
#define BCM_TRIG_FINE_DELAY_REG			0x00
#define BCM_TRIG_TO_PULSE_REG			0x12
#define BCM_UPPER_THRESHOLD_REG			0x02
#define BCM_UPPER_THRESHOLD_ALARM_REG			0x08

#define BCM_PROBE_SOURCE_SELECT_REG			0x50

/* Per channel registers are on different offsets, starting with 0x500 and 0x100
 * apart.
 * Note: For registers 0xn50 only 0x500, 0x600, 0x700 and 0x800 offsets are
 *       valid.
 */
#define BCM_CH0_OFFSET			0x500
#define BCM_CH1_OFFSET			0x600
#define BCM_CH2_OFFSET			0x700
#define BCM_CH3_OFFSET			0x800
#define BCM_CH4_OFFSET			0x900
#define BCM_CH5_OFFSET			0xA00
#define BCM_CH6_OFFSET			0xB00
#define BCM_CH7_OFFSET			0xC00
#define BCM_CH8_OFFSET			0xD00
#define BCM_CH9_OFFSET			0xE00

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
