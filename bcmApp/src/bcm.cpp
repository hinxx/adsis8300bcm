/* bcm.cpp
 *
 * This is a driver for a BCM based on Struck SIS8300 digitizer.
 *
 * Author: Hinko Kocevar
 *         ESS ERIC, Lund, Sweden
 *
 * Created:  January 22, 2017
 *
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <string>
#include <stdarg.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <iocsh.h>

#include <asynNDArrayDriver.h>
#include <epicsExport.h>

#include <SIS8300.h>
#include <bcm.h>


static const char *driverName = "Bcm";

/* asyn addresses:
 * 0 .. 9    AI channels
 * 10 .. 19  BCM channels
 */
#define BCM_ADDR_FIRST			10
#define BCM_ADDR_COUNT			10

/** Constructor for Bcm; most parameters are simply passed to SIS8300::SIS8300.
  * After calling the base class constructor this method creates a thread to compute the simulated detector data,
  * and sets reasonable default values for parameters defined in this class and SIS8300.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] devicePath The path to the /dev entry.
  * \param[in] maxAddr The maximum  number of asyn addr addresses this driver supports. 1 is minimum.
  * \param[in] numParams The number of parameters in the derived class.
  * \param[in] numSamples The initial number of samples.
  * \param[in] dataType The initial data type (NDDataType_t) of the arrays that this driver will create.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
Bcm::Bcm(const char *portName, const char *devicePath,
		int maxAddr, int numSamples, NDDataType_t dataType,
		int maxBuffers, size_t maxMemory, int priority, int stackSize)

    : SIS8300(portName, devicePath,
    		maxAddr,
			BCM_NUM_PARAMS,
			numSamples,
			dataType,
			maxBuffers, maxMemory,
			priority,
			stackSize)

{
    D(printf("%d addresses, %d parameters\n", maxAddr, BCM_NUM_PARAMS));

    /* adjust number of NDArrays we need to handle, 0 - AI, 1 - BCM */
    mNumArrays = 2;

    createParam(BcmAllAlarmsString,		asynParamInt32,	&mBcmAllAlarms);
    createParam(BcmAuxClkAlarmString,		asynParamInt32,	&mBcmAuxClkAlarm);
    createParam(BcmCalPulseString,		asynParamInt32,	&mBcmCalPulse);
    createParam(BcmClkFreqString,		asynParamInt32,	&mBcmClkFreq);
    createParam(BcmClkTrigAlarmsFirstString,		asynParamInt32,	&mBcmClkTrigAlarmsFirst);
    createParam(BcmClkTrigAlarmsLatchedString,		asynParamInt32,	&mBcmClkTrigAlarmsLatched);
    createParam(BcmDiffWarnResetString,		asynParamInt32,	&mBcmDiffWarnReset);
    createParam(BcmFlatTopTimeString,		asynParamInt32,	&mBcmFlatTopTime);
    createParam(BcmFwVersionString,		asynParamInt32,	&mBcmFwVersion);
    createParam(BcmMainClkAlarmString,		asynParamInt32,	&mBcmMainClkAlarm);
    createParam(BcmMaxPulseWidthString,		asynParamInt32,	&mBcmMaxPulseWidth);
    createParam(BcmMaxPulseWidthSrcSelString,		asynParamInt32,	&mBcmMaxPulseWidthSrcSel);
    createParam(BcmMinTrigPeriodString,		asynParamInt32,	&mBcmMinTrigPeriod);
    createParam(BcmMinTrigPeriodSrcSelString,		asynParamInt32,	&mBcmMinTrigPeriodSrcSel);
    createParam(BcmResetAlarmsString,		asynParamInt32,	&mBcmResetAlarms);
    createParam(BcmRfqTransparencyString,		asynParamInt32,	&mBcmRfqTransparency);
    createParam(BcmStatusString,		asynParamInt32,	&mBcmStatus);
    createParam(BcmSwVersionString,		asynParamInt32,	&mBcmSwVersion);
    createParam(BcmTrigPeriodString,		asynParamInt32,	&mBcmTrigPeriod);
    createParam(BcmTrigTooFastAlarmString,		asynParamInt32,	&mBcmTrigTooFastAlarm);
    createParam(BcmTrigTooLongAlarmString,		asynParamInt32,	&mBcmTrigTooLongAlarm);
    createParam(BcmTrigTooShortAlarmString,		asynParamInt32,	&mBcmTrigTooShortAlarm);
    createParam(BcmTrigWidthString,		asynParamInt32,	&mBcmTrigWidth);

    createParam(BcmAdcAlarmsFirstString,		asynParamInt32,	&mBcmAdcAlarmsFirst);
    createParam(BcmAdcAlarmsLatchedString,		asynParamInt32,	&mBcmAdcAlarmsLatched);
    createParam(BcmAdcOffsetString,		asynParamInt32,	&mBcmAdcOffset);
    createParam(BcmAdcOffsErrorAvgString,		asynParamInt32,	&mBcmAdcOffsErrorAvg);
    createParam(BcmAdcOffsErrorIntegString,		asynParamInt32,	&mBcmAdcOffsErrorInteg);
    createParam(BcmAdcScaleString,		asynParamInt32,	&mBcmAdcScale);
    createParam(BcmAdcStuckAlarmString,		asynParamInt32,	&mBcmAdcStuckAlarm);
    createParam(BcmCalPulseAmplEarlyString,		asynParamInt32,	&mBcmCalPulseAmplEarly);
    createParam(BcmCalPulseAmplLateString,		asynParamInt32,	&mBcmCalPulseAmplLate);
    createParam(BcmDroopRateString,		asynParamInt32,	&mBcmDroopRate);
    createParam(BcmDroopBaselineString,		asynParamInt32,	&mBcmDroopBaseline);
    createParam(BcmDroopErrorString,		asynParamInt32,	&mBcmDroopError);
    createParam(BcmErrantThresholdString,		asynParamInt32,	&mBcmErrantThreshold);
    createParam(BcmErrantThresholdAlarmString,		asynParamInt32,	&mBcmErrantThresholdAlarm);
    createParam(BcmFlatTopChargeString,		asynParamInt32,	&mBcmFlatTopCharge);
    createParam(BcmLocationMebtString,		asynParamInt32,	&mBcmLocationMebt);
    createParam(BcmLocationRfqString,		asynParamInt32,	&mBcmLocationRfq);
    createParam(BcmLowerThresholdString,		asynParamInt32,	&mBcmLowerThreshold);
    createParam(BcmLowerThresholdAlarmString,		asynParamInt32,	&mBcmLowerThresholdAlarm);
    createParam(BcmNoiseFilterString,		asynParamInt32,	&mBcmNoiseFilter);
    createParam(BcmOverflowAlarmString,		asynParamInt32,	&mBcmOverflowAlarm);
    createParam(BcmPulseChargeString,		asynParamInt32,	&mBcmPulseCharge);
    createParam(BcmPulsePastLimitAlarmString,		asynParamInt32,	&mBcmPulsePastLimitAlarm);
    createParam(BcmPulsePastTriggerAlarmString,		asynParamInt32,	&mBcmPulsePastTriggerAlarm);
    createParam(BcmPulseWidthString,		asynParamInt32,	&mBcmPulseWidth);
    createParam(BcmTrigFineDelayString,		asynParamInt32,	&mBcmTrigFineDelay);
    createParam(BcmTrigToPulseString,		asynParamInt32,	&mBcmTrigToPulse);
    createParam(BcmUnderflowAlarmString,		asynParamInt32,	&mBcmUnderflowAlarm);
    createParam(BcmUpperThresholdString,		asynParamInt32,	&mBcmUpperThreshold);
    createParam(BcmUpperThresholdAlarmString,		asynParamInt32,	&mBcmUpperThresholdAlarm);

    createParam(BcmProbeSourceSelectString,		asynParamInt32,	&mBcmProbeSourceSelect);

    this->lock();
    initDevice();
    this->unlock();

	I(printf("Init done...\n"));
}

Bcm::~Bcm() {
	I(printf("Shutdown complete!\n"));
}

int Bcm::initDevice()
{
    unsigned int firmwareVersion;

    D(printf("Enter\n"));

	SIS8300DRV_CALL_RET("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, BCM_FW_VERSION_REG, &firmwareVersion));

	setIntegerParam(mBcmFwVersion, firmwareVersion >> 16);
	callParamCallbacks(0);

	I(printf("BCM firmware version 0x%X\n", firmwareVersion >> 16));

	return 0;
}

template <typename epicsType> int Bcm::convertAIArraysT(int aich)
{
    int numAiSamples;
    epicsType *pData, *pVal;
    epicsUInt16 *pRaw, *pChRaw;
    int i;
    double convFactor, convOffset;
    bool negative;

	D(printf("Enter\n"));

    getIntegerParam(mSISNumAiSamples, &numAiSamples);

    /* local NDArray is for raw AI data samples */
    if (! mRawDataArray) {
    	return -1;
    }
    pRaw = (epicsUInt16 *)mRawDataArray->pData;

    /* 0th NDArray is for converted AI data samples */
    if (! this->pArrays[0]) {
    	return -1;
    }
    pData = (epicsType *)this->pArrays[0]->pData;
	pChRaw = pRaw + (aich * numAiSamples);
	pVal = pData + aich;
	getDoubleParam(aich, mSISConvFactor, &convFactor);
	getDoubleParam(aich, mSISConvOffset, &convOffset);

	char fname[32];
	sprintf(fname, "/tmp/%d.txt", aich);
	FILE *fp = fopen(fname, "w");
	D(printf("CH %d [%d] ", aich, numAiSamples));
	for (i = 0; i < numAiSamples; i++) {
		negative = (*(pChRaw + i) & (1 << 15)) != 0;
		if (negative) {
			*pVal = (epicsType)((double)(*(pChRaw + i) | ~((1 << 16) - 1)) * convFactor + convOffset);
		} else {
			*pVal = (epicsType)((double)(*(pChRaw + i)) * convFactor + convOffset);
		}

//		printf("%f ", (double)*pVal);
        fprintf(fp, "%d\t\t%f\n", *(pChRaw + i), (double)*pVal);
		pVal += SIS8300DRV_NUM_AI_CHANNELS;
	}
	D0(printf("\n"));
	fclose(fp);

    return 0;
}

template <typename epicsType> int Bcm::convertBCMArraysT(int aich)
{
    int numAiSamples;
    int numBCMSamples;
    epicsType *pData, *pVal;
    epicsUInt16 *pRaw, *pChRaw;
    int i, j, k;
//    double converted;
    double convFactor, convOffset;
    bool negative;

	D(printf("Enter\n"));

    getIntegerParam(mSISNumAiSamples, &numAiSamples);
    numBCMSamples = (numAiSamples / 8) - 2;

    /* local NDArray is for raw AI data samples */
    if (! mRawDataArray) {
    	return -1;
    }
    pRaw = (epicsUInt16 *)mRawDataArray->pData;
    /* 1st NDArray is for BPM 1 data samples */
    if (! this->pArrays[1]) {
    	return -1;
    }
    pData = (epicsType *)this->pArrays[1]->pData;

	i = 0;
	j = 0;
	pChRaw = pRaw + (aich * numAiSamples);
	pVal = pData;

	D(printf("CH %d [%d] BCM samples %d\n", aich, numAiSamples, numBCMSamples));

	char fname[32];
	sprintf(fname, "/tmp/bcm_0_%d.txt", aich);
    FILE *fp = NULL;
    if (aich == 0) {
    	fp = fopen(fname, "w");
    }
	while (i < numAiSamples) {
		/* since will always take less IQ samples from raw data than available
		 * we need to bail out when desired amount was collected */
		if (j == numBCMSamples) {
			break;
		}

		assert(i < numAiSamples);

		if (aich == 0) {
			getDoubleParam(aich, mSISConvFactor, &convFactor);
			getDoubleParam(aich, mSISConvOffset, &convOffset);

			/* BCM 1 .. 8 data is here */
			for (k = 0; k < 8; k++) {
				negative = (*(pChRaw + k) & (1 << 15)) != 0;
				if (negative) {
					*(pVal + k) = (epicsType)((double)(*(pChRaw + k) | ~((1 << 16) - 1)) * convFactor + convOffset);
				} else {
					*(pVal + k) = (epicsType)((double)(*(pChRaw + k)) * convFactor + convOffset);
				}
			}
            if (aich == 0) {
                fprintf(fp, "%f\n", (double)*(pVal));
            }
		} else if (aich == 1) {
			getDoubleParam(aich, mSISConvFactor, &convFactor);
			getDoubleParam(aich, mSISConvOffset, &convOffset);

			/* BCM 9 .. 10 data is here */
			for (k = 0; k < 2; k++) {
				negative = (*(pChRaw + k + 8) & (1 << 15)) != 0;
				if (negative) {
					*(pVal + k + 8) = (epicsType)((double)(*(pChRaw + k) | ~((1 << 16) - 1)) * convFactor + convOffset);
				} else {
					*(pVal + k + 8) = (epicsType)((double)(*(pChRaw + k)) * convFactor + convOffset);
				}
			}
		} else {
			E(printf("Should not be here!!!\n"));
			assert(1 == 0);
		}

		/* adjust raw AI offset */
		i += 8;
		/* adjust raw AI data pointer */
		pChRaw += 8;
		/* adjust BCM offset for all channels */
		j++;
		/* adjust BCM data pointer */
		pVal += BCM_NUM_CHANNELS;
		}
	    if ((aich == 0) && fp) {
    		fclose(fp);
    	}

    return 0;
}

template <typename epicsType> int Bcm::convertArraysT()
{
    size_t dims[2];
    int numAiSamples;
    int numBCMSamples;
    NDDataType_t dataType;
    epicsType *pData;
    int aich;
    int ret;

	D(printf("Enter\n"));

    getIntegerParam(NDDataType, (int *)&dataType);
    getIntegerParam(mSISNumAiSamples, &numAiSamples);
    numBCMSamples = (numAiSamples / 8) - 2;

    /* local NDArray is for raw AI data samples */
    if (! mRawDataArray) {
    	return -1;
    }

    /* converted AI data samples of all channel are interleaved */
    dims[0] = SIS8300_NUM_CHANNELS;
    dims[1] = numAiSamples;

    /* 0th NDArray is for converted AI data samples */
    if (this->pArrays[0]) {
    	this->pArrays[0]->release();
    }
    this->pArrays[0] = pNDArrayPool->alloc(2, dims, dataType, 0, 0);
    pData = (epicsType *)this->pArrays[0]->pData;
    memset(pData, 0, SIS8300_NUM_CHANNELS * numAiSamples * sizeof(epicsType));

    /* converted BCM data samples of all channels are interleaved */
    dims[0] = BCM_NUM_CHANNELS;
    dims[1] = numBCMSamples;

    /* 1st NDArray is for converted BCM data samples */
    if (this->pArrays[1]) {
    	this->pArrays[1]->release();
    }
    this->pArrays[1] = pNDArrayPool->alloc(2, dims, dataType, 0, 0);
    pData = (epicsType *)this->pArrays[1]->pData;
    memset(pData, 0, BCM_NUM_CHANNELS * numBCMSamples * sizeof(epicsType));

    for (aich = 0; aich < SIS8300_NUM_CHANNELS; aich++) {
        if (!(mChannelMask & (1 << aich))) {
            continue;
        }

		/* AI data is here */
		ret = convertAIArraysT<epicsType>(aich);
        if (ret) {
            return ret;
        }
		if (aich < 2) {
    		/* BCM data is here */
    		ret = convertBCMArraysT<epicsType>(aich);
    		if (ret) {
    			return ret;
    		}
		}
    }

    return 0;
}

int Bcm::acquireArrays()
{
    int dataType;
    int ret;

	D(printf("Enter\n"));

    ret = acquireRawArrays();
    if (ret) {
    	return ret;
    }

    getIntegerParam(NDDataType, &dataType);
    switch (dataType) {
        case NDInt8:
            return convertArraysT<epicsInt8>();
            break;
        case NDUInt8:
        	return convertArraysT<epicsUInt8>();
            break;
        case NDInt16:
        	return convertArraysT<epicsInt16>();
            break;
        case NDUInt16:
        	return convertArraysT<epicsUInt16>();
            break;
        case NDInt32:
        	return convertArraysT<epicsInt32>();
            break;
        case NDUInt32:
        	return convertArraysT<epicsUInt32>();
            break;
        case NDFloat32:
        	return convertArraysT<epicsFloat32>();
            break;
        case NDFloat64:
        	return convertArraysT<epicsFloat64>();
            break;
        default:
        	return -1;
        	break;
    }
}

int Bcm::updateRegisterParameter(int index, int reg, int mask, int readFirst)
{
	return updateRegisterParameter(0, index, reg, mask, readFirst);
}

int Bcm::updateRegisterParameter(int list, int index, int reg, int mask, int readFirst)
{
	int changed;
	asynStatus status;
	int value;
	unsigned int regValue;
	int ret = 0;

	/* adjust register address according to the list == BCM channel. */
	if (list) {
		reg = reg + BCM_CH0_OFFSET + ((list - BCM_ADDR_FIRST) * 0x100);
	}

	D(printf("list %d, index %d, reg %d, mask %x, read? %d\n", list, index, reg, mask, readFirst));

	status = hasParamValueChanged(index, &changed);
	if (status) {
		return -1;
	}

	if (! changed) {
		/* nothing to do .. */
		return 0;
	}

	/* value has changed, update the FPGA register */
	status = getIntegerParam(index, &value);
	if (status) {
		return -1;
	}
	regValue = 0;
	if (readFirst) {
		ret = SIS8300DRV_CALL("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, reg, &regValue));
		if (ret) {
			return -1;
		}
		if (mask) {
			regValue &= ~mask;
			regValue |= (value & mask);
		}
	} else {
		if (mask) {
			regValue = (value & mask);
		} else {
			regValue = value;
		}
	}
	ret = SIS8300DRV_CALL("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, reg, regValue));
	if (ret) {
		return -1;
	}

	/* clear the value has changed flag */
	status = clearParamValueChanged(index);
	if (status) {
		return -1;
	}

	return 0;
}

int Bcm::refreshRegisterParameter(int index, int reg, int mask)
{
	return refreshRegisterParameter(0, index, reg, mask);
}

int Bcm::refreshRegisterParameter(int list, int index, int reg, int mask)
{
	asynStatus status;
	int value;
	unsigned int regValue;
    const char *name;
	int ret = 0;

	/* adjust register address according to the list == BCM channel. */
	if (list) {
		reg = reg + BCM_CH0_OFFSET + ((list - BCM_ADDR_FIRST) * 0x100);
	}

	D(printf("list %d, index %d, reg %d, mask %x\n", list, index, reg, mask));

	/* read FPGA register and update the parameter */
	ret = SIS8300DRV_CALL("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, reg, &regValue));
	if (ret) {
		return -1;
	}
	value = regValue;
    getParamName(index, &name);
	D(printf("Setting '%s' %d (%d) = %d\n", name, index, list, value));

	status = setIntegerParam(list, index, value);
	if (status) {
		return -1;
	}

	return 0;
}

int Bcm::deviceDone()
{
	int i;
	int ret = 0;

	D(printf("Enter\n"));

	ret |= refreshRegisterParameter(mBcmStatus, BCM_STATUS_REG, 0x3F);
	ret |= refreshRegisterParameter(mBcmClkFreq, BCM_CLK_FREQ_REG, 0xFFFFF);
	ret |= refreshRegisterParameter(mBcmTrigPeriod, BCM_TRIG_PERIOD_REG, 0xFFFFFFFF);
	ret |= refreshRegisterParameter(mBcmTrigWidth, BCM_TRIG_WIDTH_REG, 0xFFFFF);
	ret |= refreshRegisterParameter(mBcmFlatTopTime, BCM_FLAT_TOP_TIME_REG, 0xFFFFF);
	ret |= refreshRegisterParameter(mBcmClkTrigAlarmsLatched, BCM_CLK_TRIG_ALARMS_LATCHED_REG, 0x1F);
	ret |= refreshRegisterParameter(mBcmClkTrigAlarmsFirst, BCM_CLK_TRIG_ALARMS_FIRST_REG, 0x1F);

	/* Do callbacks so higher layers see any changes */
    callParamCallbacks(0);

	for (i = BCM_ADDR_FIRST; i < (BCM_ADDR_FIRST + BCM_ADDR_COUNT); i++) {
		ret |= refreshRegisterParameter(i, mBcmAdcOffsErrorAvg, BCM_ADC_OFFS_ERROR_AVG_REG, 0xFFFF);
		ret |= refreshRegisterParameter(i, mBcmAdcOffsErrorInteg, BCM_ADC_OFFS_ERROR_INTEG_REG, 0xFFFFFF);
		ret |= refreshRegisterParameter(i, mBcmTrigToPulse, BCM_TRIG_TO_PULSE_REG, 0x7FF);
		ret |= refreshRegisterParameter(i, mBcmDroopError, BCM_DROOP_ERROR_REG, 0xFFFF);
		ret |= refreshRegisterParameter(i, mBcmPulseWidth, BCM_PULSE_WIDTH_REG, 0xFFFFF);
		ret |= refreshRegisterParameter(i, mBcmPulseCharge, BCM_PULSE_CHARGE_REG, 0xFFFFFFFF);
		ret |= refreshRegisterParameter(i, mBcmFlatTopCharge, BCM_FLAT_TOP_CHARGE_REG, 0xFFFFFFFF);
		ret |= refreshRegisterParameter(i, mBcmCalPulseAmplEarly, BCM_CAL_PULSE_AMPL_EARLY_REG, 0xFFFF);
		ret |= refreshRegisterParameter(i, mBcmCalPulseAmplLate, BCM_CAL_PULSE_AMPL_LATE_REG, 0xFFFF);
		ret |= refreshRegisterParameter(i, mBcmAdcAlarmsLatched, BCM_ADC_ALARMS_LATCHED_REG, 0xFF);
		ret |= refreshRegisterParameter(i, mBcmAdcAlarmsFirst, BCM_ADC_ALARMS_FIRST_REG, 0xFF);

	    /* Do callbacks so higher layers see any changes */
	    callParamCallbacks(i);
	}

	return ret;
}

int Bcm::updateParameters()
{
	int i;
	int ret = 0;

	D(printf("Enter\n"));

	ret |= updateRegisterParameter(mBcmMinTrigPeriodSrcSel, BCM_MIN_TRIG_PERIOD_SRC_SEL_REG, 0x1, 1);
	ret |= updateRegisterParameter(mBcmMaxPulseWidthSrcSel, BCM_MAX_PULSE_WIDTH_SRC_SEL_REG, 0x2, 1);
	ret |= updateRegisterParameter(mBcmRfqTransparency, BCM_RFQ_TRANSPARENCY_REG, 0xFFFF, 0);
	ret |= updateRegisterParameter(mBcmDiffWarnReset, BCM_DIFF_WARN_RESET_REG, 0x1, 0);
	ret |= updateRegisterParameter(mBcmMinTrigPeriod, BCM_MIN_TRIG_PERIOD_REG, 0xFFFF, 0);
	ret |= updateRegisterParameter(mBcmMaxPulseWidth, BCM_MAX_PULSE_WIDTH_REG, 0x1FFF, 0);
	ret |= updateRegisterParameter(mBcmCalPulse, BCM_CAL_PULSE_REG, 0x1, 0);
	ret |= updateRegisterParameter(mBcmResetAlarms, BCM_RESET_ALARMS_REG, 0x1, 0);
	ret |= updateRegisterParameter(mBcmAllAlarms, BCM_ALL_ALARMS_REG, 0x1, 0);
	ret |= updateRegisterParameter(mBcmAuxClkAlarm, BCM_AUX_CLK_ALARM_REG, 0x1, 1);
	ret |= updateRegisterParameter(mBcmMainClkAlarm, BCM_MAIN_CLK_ALARM_REG, 0x2, 1);
	ret |= updateRegisterParameter(mBcmTrigTooShortAlarm, BCM_TRIG_TOO_SHORT_ALARM_REG, 0x4, 1);
	ret |= updateRegisterParameter(mBcmTrigTooLongAlarm, BCM_TRIG_TOO_LONG_ALARM_REG, 0x8, 1);
	ret |= updateRegisterParameter(mBcmTrigTooFastAlarm, BCM_TRIG_TOO_FAST_ALARM_REG, 0x10, 1);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(0);

	D(printf("ret = %d\n", ret));

	for (i = BCM_ADDR_FIRST; i < (BCM_ADDR_FIRST + BCM_ADDR_COUNT); i++) {
		ret |= updateRegisterParameter(i, mBcmTrigFineDelay, BCM_TRIG_FINE_DELAY_REG, 0x7FF, 0);
		ret |= updateRegisterParameter(i, mBcmAdcScale, BCM_ADC_SCALE_REG, 0xFFFF, 0);
		ret |= updateRegisterParameter(i, mBcmUpperThreshold, BCM_UPPER_THRESHOLD_REG, 0xFFFF, 0);
		ret |= updateRegisterParameter(i, mBcmLowerThreshold, BCM_LOWER_THRESHOLD_REG, 0xFFFF, 0);
		ret |= updateRegisterParameter(i, mBcmErrantThreshold, BCM_ERRANT_THRESHOLD_REG, 0xFFFF, 0);
		ret |= updateRegisterParameter(i, mBcmAdcOffset, BCM_ADC_OFFSET_REG, 0xFFFF, 0);
		ret |= updateRegisterParameter(i, mBcmDroopRate, BCM_DROOP_RATE_REG, 0xFFFF, 0);
		ret |= updateRegisterParameter(i, mBcmDroopBaseline, BCM_DROOP_BASELINE_REG, 0x1, 1);
		ret |= updateRegisterParameter(i, mBcmNoiseFilter, BCM_NOISE_FILTER_REG, 0x2, 1);
		ret |= updateRegisterParameter(i, mBcmLocationMebt, BCM_LOCATION_MEBT_REG, 0x10, 1);
		ret |= updateRegisterParameter(i, mBcmLocationRfq, BCM_LOCATION_RFQ_REG, 0x20, 1);
		ret |= updateRegisterParameter(i, mBcmUpperThresholdAlarm, BCM_UPPER_THRESHOLD_ALARM_REG, 0x1, 1);
		ret |= updateRegisterParameter(i, mBcmLowerThresholdAlarm, BCM_LOWER_THRESHOLD_ALARM_REG, 0x2, 1);
		ret |= updateRegisterParameter(i, mBcmErrantThresholdAlarm, BCM_ERRANT_THRESHOLD_ALARM_REG, 0x4, 1);
		ret |= updateRegisterParameter(i, mBcmPulsePastTriggerAlarm, BCM_PULSE_PAST_TRIGGER_ALARM_REG, 0x8, 1);
		ret |= updateRegisterParameter(i, mBcmPulsePastLimitAlarm, BCM_PULSE_PAST_LIMIT_ALARM_REG, 0x10, 1);
		ret |= updateRegisterParameter(i, mBcmOverflowAlarm, BCM_OVERFLOW_ALARM_REG, 0x20, 1);
		ret |= updateRegisterParameter(i, mBcmUnderflowAlarm, BCM_UNDERFLOW_ALARM_REG, 0x40, 1);
		ret |= updateRegisterParameter(i, mBcmAdcStuckAlarm, BCM_ADC_STUCK_ALARM_REG, 0x80, 1);

	    /* Do callbacks so higher layers see any changes */
	    callParamCallbacks(i);
	}

	D(printf("ret = %d\n", ret));
	return ret;
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus Bcm::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int addr;
    const char *name;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    getParamName(function, &name);
    D(printf("Enter '%s' %d (%d) = %d\n", name, function, addr, value));
 
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(addr, function, value);

    /* If this parameter belongs to a base class call its method */
    if (function < BCM_FIRST_PARAM) {
    	status = SIS8300::writeInt32(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr);

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeInt32 error, status=%d function=%d, value=%d\n",
              driverName, status, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeInt32: function=%d, value=%d\n",
              driverName, function, value);
    }

    return status;
}

/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void Bcm::report(FILE *fp, int details)
{
    fprintf(fp, "BCM\n");
    if (details > 0) {
    }

    /* Invoke the base class method */
    SIS8300::report(fp, details);
}

/** Configuration command, called directly or from iocsh */
extern "C" int BcmConfig(const char *portName, const char *devicePath,
		int maxAddr, int numSamples, int dataType, int maxBuffers, int maxMemory,
		int priority, int stackSize)
{
    new Bcm(portName, devicePath,
    		maxAddr,
    		numSamples,
			(NDDataType_t)dataType,
			(maxBuffers < 0) ? 0 : maxBuffers,
			(maxMemory < 0) ? 0 : maxMemory,
			priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg configArg0 = {"Port name",     iocshArgString};
static const iocshArg configArg1 = {"Device path",   iocshArgString};
static const iocshArg configArg2 = {"# channels",    iocshArgInt};
static const iocshArg configArg3 = {"# samples",     iocshArgInt};
static const iocshArg configArg4 = {"Data type",     iocshArgInt};
static const iocshArg configArg5 = {"maxBuffers",    iocshArgInt};
static const iocshArg configArg6 = {"maxMemory",     iocshArgInt};
static const iocshArg configArg7 = {"priority",      iocshArgInt};
static const iocshArg configArg8 = {"stackSize",     iocshArgInt};
static const iocshArg * const BcmConfigArgs[] = {&configArg0,
												 &configArg1,
												 &configArg2,
												 &configArg3,
												 &configArg4,
												 &configArg5,
												 &configArg6,
												 &configArg7,
												 &configArg8};
static const iocshFuncDef configBcm = {"BcmConfig", 9, BcmConfigArgs};
static void configBcmCallFunc(const iocshArgBuf *args)
{
    BcmConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
    		args[4].ival, args[5].ival, args[6].ival, args[7].ival, args[8].ival);
}


static void BcmRegister(void)
{
    iocshRegister(&configBcm, configBcmCallFunc);
}

extern "C" {
epicsExportRegistrar(BcmRegister);
}
