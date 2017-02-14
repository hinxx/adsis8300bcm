/* ADSIS8300bcm.h
 *
 * This is a driver for a Struck SIS8300 BCM digitizer.
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

#define ADSIS8300BCM_NUM_CHANNELS      1

#define ADSIS8300BCM_IRQ_WAIT_TIME     0

/* maximum number of BCM registers that we support */
#define ADSIS8300BCM_NREGISTERS        1000

/** Struck SIS8300 BCM driver; does 1-D waveforms on 1 channel.
  * Inherits from ADSIS8300 */
class epicsShareClass ADSIS8300bcm : public ADSIS8300 {
public:
	ADSIS8300bcm(const char *portName, const char *devicePath,
			int maxAddr, int numTimePoints, NDDataType_t dataType,
			int maxBuffers, size_t maxMemory, int priority, int stackSize);
	~ADSIS8300bcm();

    /* These are the methods that we override from asynNDArrayDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo,
                                     const char **pptypeName, size_t *psize);
    virtual void report(FILE *fp, int details);

protected:
    virtual int deviceDone();

    /* System wide parameters */
    int mRegisters[ADSIS8300BCM_NREGISTERS];

private:
    /* Our data */
    unsigned int mRegisterIndex;
};

#define NUM_SIS8300BCM_PARAMS ((int)(ADSIS8300BCM_NREGISTERS))
