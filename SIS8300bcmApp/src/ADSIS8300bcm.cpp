/* ADSIS8300bcm.cpp
 *
 * This is a driver for a Struck SIS8300 BCM digitizer.
 *
 * Author: Hinko Kocevar
 *         ESS ERIC, Lund, Sweden
 *
 * Created:  September 22, 2016
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
#include <ADSIS8300bcm.h>


static const char *driverName = "ADSIS8300bcm";

/* asyn addresses:
 * 0 .. 9    AI channels
 * 10        BCM channels
 */
#define SIS8300BCM_BCM_ADDR		10

/** Constructor for SIS8300bcm; most parameters are simply passed to ADSIS8300::ADSIS8300.
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
ADSIS8300bcm::ADSIS8300bcm(const char *portName, const char *devicePath,
		int maxAddr, int numSamples, NDDataType_t dataType,
		int maxBuffers, size_t maxMemory, int priority, int stackSize)

    : ADSIS8300(portName, devicePath,
    		maxAddr,
    		NUM_SIS8300BCM_PARAMS,
			numSamples,
			dataType,
			maxBuffers, maxMemory,
			priority,
			stackSize)

{
    D(printf("%d addresses, %d parameters\n", maxAddr, NUM_SIS8300BCM_PARAMS));

    this->mRegisterIndex = 0;

	I(printf("Init done...\n"));
}

ADSIS8300bcm::~ADSIS8300bcm() {
	D(printf("Shutdown and freeing up memory...\n"));

	this->lock();
	D(printf("Data thread is already down!\n"));
	destroyDevice();

	this->unlock();
	I(printf("Shutdown complete!\n"));
}

asynStatus ADSIS8300bcm::drvUserCreate(asynUser *pasynUser, const char *drvInfo,
                                     const char **pptypeName, size_t *psize) {

	asynStatus status;
	int index;
	epicsInt32 val;

	D(printf("Enter for '%s'\n", drvInfo));

	status = findParam(drvInfo, &index);
	if (status && strlen(drvInfo)) {
        /* Check we have allocated enough space */
        if (mRegisterIndex > ADSIS8300BCM_NREGISTERS) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: Not enough space allocated to store all features, increase NFEATURES\n",
                        driverName, __func__);
            return asynError;
        }
		status = createParam(drvInfo, asynParamInt32, &(mRegisters[mRegisterIndex]));
		if (status) {
			return status;
		}
		status = ADSIS8300::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
		if (status) {
			return status;
		}
		status = readInt32(pasynUser, &val);
		if (status) {
			return status;
		}
		mRegisterIndex++;

		return asynSuccess;
	}

	// Just return baseclass result
    return ADSIS8300::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ADSIS8300bcm::writeInt32(asynUser *pasynUser, epicsInt32 value)
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

asynStatus ADSIS8300bcm::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int addr;
    const char *name;
    int reg, ret;
    unsigned int val;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    getParamName(function, &name);

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
void ADSIS8300bcm::report(FILE *fp, int details)
{
    fprintf(fp, "Struck SIS8300 based BCM\n");
    if (details > 0) {
    }

    /* Invoke the base class method */
    ADSIS8300::report(fp, details);
}

/** Configuration command, called directly or from iocsh */
extern "C" int SIS8300BcmConfig(const char *portName, const char *devicePath,
		int maxAddr, int numSamples, int dataType, int maxBuffers, int maxMemory,
		int priority, int stackSize)
{
    new ADSIS8300bcm(portName, devicePath,
    		maxAddr,
    		numSamples,
			(NDDataType_t)dataType,
			(maxBuffers < 0) ? 0 : maxBuffers,
			(maxMemory < 0) ? 0 : maxMemory,
			priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg SIS8300BcmConfigArg0 = {"Port name",     iocshArgString};
static const iocshArg SIS8300BcmConfigArg1 = {"Device path",   iocshArgString};
static const iocshArg SIS8300BcmConfigArg2 = {"# channels",    iocshArgInt};
static const iocshArg SIS8300BcmConfigArg3 = {"# samples",     iocshArgInt};
static const iocshArg SIS8300BcmConfigArg4 = {"Data type",     iocshArgInt};
static const iocshArg SIS8300BcmConfigArg5 = {"maxBuffers",    iocshArgInt};
static const iocshArg SIS8300BcmConfigArg6 = {"maxMemory",     iocshArgInt};
static const iocshArg SIS8300BcmConfigArg7 = {"priority",      iocshArgInt};
static const iocshArg SIS8300BcmConfigArg8 = {"stackSize",     iocshArgInt};
static const iocshArg * const SIS8300BcmConfigArgs[] = {&SIS8300BcmConfigArg0,
                                                     &SIS8300BcmConfigArg1,
													 &SIS8300BcmConfigArg2,
													 &SIS8300BcmConfigArg3,
													 &SIS8300BcmConfigArg4,
													 &SIS8300BcmConfigArg5,
													 &SIS8300BcmConfigArg6,
													 &SIS8300BcmConfigArg7,
													 &SIS8300BcmConfigArg8};
static const iocshFuncDef configSIS8300Bcm = {"SIS8300BcmConfig", 9, SIS8300BcmConfigArgs};
static void configSIS8300BcmCallFunc(const iocshArgBuf *args)
{
    SIS8300BcmConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
    		args[4].ival, args[5].ival, args[6].ival, args[7].ival, args[8].ival);
}


static void SIS8300BcmRegister(void)
{
    iocshRegister(&configSIS8300Bcm, configSIS8300BcmCallFunc);
}

extern "C" {
epicsExportRegistrar(SIS8300BcmRegister);
}
