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

#include <ADSIS8300.h>
#include <bcm.h>


static const char *driverName = "Bcm";

/* asyn addresses:
 * 0 .. 9    AI channels
 * 10 .. 19  BCM channels
 * 20 .. 23  Probe channels
 */
#define BCM_ADDR		10

/** Constructor for Bcm; most parameters are simply passed to ADSIS8300::ADSIS8300.
  * After calling the base class constructor this method creates a thread to compute the simulated detector data,
  * and sets reasonable default values for parameters defined in this class and ADSIS8300.
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

    : ADSIS8300(portName, devicePath,
    		maxAddr,
    		NUM_BCM_PARAMS,
			numSamples,
			dataType,
			maxBuffers, maxMemory,
			priority,
			stackSize)

{
    D(printf("%d addresses, %d parameters\n", maxAddr, NUM_BCM_PARAMS));

//    this->mRegisterIndex = 0;

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
    createParam(BcmMinTrigPeriodString,		asynParamInt32,	&mBcmMinTrigPeriod);
    createParam(BcmResetAlarmsString,		asynParamInt32,	&mBcmResetAlarms);
    createParam(BcmRfqTransparencyString,		asynParamInt32,	&mBcmRfqTransparency);
    createParam(BcmStatusString,		asynParamInt32,	&mBcmStatus);
    createParam(BcmSwVersionString,		asynParamInt32,	&mBcmSwVersion);
    createParam(BcmTrigPeriodString,		asynParamInt32,	&mBcmTrigPeriod);
    createParam(BcmTrigPeriodSrcSelString,		asynParamInt32,	&mBcmTrigPeriodSrcSel);
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
    createParam(BcmCalPulseAmplEarlyString,		asynParamInt32,	&mBcmCalPulseAmplEarly);
    createParam(BcmCalPulseAmplLateString,		asynParamInt32,	&mBcmCalPulseAmplLate);
    createParam(BcmDrooprateString,		asynParamInt32,	&mBcmDrooprate);
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
    createParam(BcmStuckAlarmString,		asynParamInt32,	&mBcmStuckAlarm);
    createParam(BcmTrigFineDelayString,		asynParamInt32,	&mBcmTrigFineDelay);
    createParam(BcmTrigToPulseString,		asynParamInt32,	&mBcmTrigToPulse);
    createParam(BcmUpperThresholdString,		asynParamInt32,	&mBcmUpperThreshold);
    createParam(BcmUpperThresholdAlarmString,		asynParamInt32,	&mBcmUpperThresholdAlarm);

    createParam(BcmProbeSourceSelectString,		asynParamInt32,	&mBcmProbeSourceSelect);

    this->lock();
    initDevice();
    this->unlock();

	I(printf("Init done...\n"));
}

Bcm::~Bcm() {
	D(printf("Shutdown and freeing up memory...\n"));

	this->lock();
	D(printf("Data thread is already down!\n"));
	destroyDevice();

	this->unlock();
	I(printf("Shutdown complete!\n"));
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
    int reg, ret;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    getParamName(function, &name);
    D(printf("Enter '%s' %d (%d) = %d\n", name, function, addr, value));
 
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(addr, function, value);

#if 0
    if (function >= mRegisters[0]) {
		ret = sscanf(name, "%*s %X", &reg);
		if (ret == 1) {
			/* This is our BCM register access */
			ret = SIS8300DRV_CALL("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, reg, value));
			if (ret) {
				status = asynError;
			}
		}
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < mRegisters[0]) {
        	status = ADSIS8300::writeInt32(pasynUser, value);
        }
    }
#endif

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

asynStatus Bcm::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int addr;
    const char *name;
    int reg, ret;
    unsigned int val;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    getParamName(function, &name);

#if 0
    if (function >= mRegisters[0]) {
		D(printf("Enter '%s' %d (%d)\n", name, function, addr));

		ret = sscanf(name, "%*s %X", &reg);
		if (ret == 1) {
			/* This is our BCM register access */
			ret = SIS8300DRV_CALL("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, reg, &val));
			if (ret) {
				status = asynError;
			} else {
				status = setIntegerParam(addr, function, val);
				D(printf("Setting '%s' %d (%d) = %d\n", name, function, addr, val));
			}
		}
    }
#endif

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:readInt32 error, status=%d function=%d, value=%d\n",
              driverName, status, function, *value);
        return status;
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:readInt32: function=%d, value=%d\n",
              driverName, function, *value);
    }

    // Call base class
    status = ADSIS8300::readInt32(pasynUser, value);
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
    ADSIS8300::report(fp, details);
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
