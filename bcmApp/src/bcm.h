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

#define BCM_NUM_CHANNELS      1

#define BCM_IRQ_WAIT_TIME     0

/* maximum number of BCM registers that we support */
#define BCM_NREGISTERS        1000

class epicsShareClass Bcm : public ADSIS8300 {
public:
	Bcm(const char *portName, const char *devicePath,
			int maxAddr, int numTimePoints, NDDataType_t dataType,
			int maxBuffers, size_t maxMemory, int priority, int stackSize);
	~Bcm();

    /* These are the methods that we override from asynNDArrayDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo,
                                     const char **pptypeName, size_t *psize);
    virtual void report(FILE *fp, int details);

protected:
    /* System wide parameters */
    int mRegisters[BCM_NREGISTERS];

private:
    /* Our data */
    unsigned int mRegisterIndex;
};

#define NUM_BCM_PARAMS ((int)(BCM_NREGISTERS))
