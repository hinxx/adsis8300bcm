< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/bcmDemoApp.dbd")
bcmDemoApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "BCM:")
# The port name for the detector
epicsEnvSet("PORT",   "BCM")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "64")
# The maximum number of time series points in the NDPluginTimeSeries plugin
epicsEnvSet("TSPOINTS", "600000")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

epicsEnvSet("AICH0",      "AI0")
epicsEnvSet("AICH1",      "AI1")
epicsEnvSet("AICH2",      "AI2")
epicsEnvSet("AICH3",      "AI3")
epicsEnvSet("AICH4",      "AI4")
epicsEnvSet("AICH5",      "AI5")
epicsEnvSet("AICH6",      "AI6")
epicsEnvSet("AICH7",      "AI7")
epicsEnvSet("AICH8",      "AI8")
epicsEnvSet("AICH9",      "AI9")

epicsEnvSet("BCM0",       "BCM0")
epicsEnvSet("BCM1",       "BCM1")
epicsEnvSet("BCM2",       "BCM2")
epicsEnvSet("BCM3",       "BCM3")
epicsEnvSet("BCM4",       "BCM4")
epicsEnvSet("BCM5",       "BCM5")
epicsEnvSet("BCM6",       "BCM6")
epicsEnvSet("BCM7",       "BCM7")
epicsEnvSet("BCM8",       "BCM8")
epicsEnvSet("BCM9",       "BCM9")

epicsEnvSet("PROBE0",     "PROBE0")
epicsEnvSet("PROBE1",     "PROBE1")
epicsEnvSet("PROBE2",     "PROBE2")
epicsEnvSet("PROBE3",     "PROBE3")

# This is sum of AI and BCM asyn addresses
# ADDR 0 .. 9 are for AI data
# ADDR 10 .. 19  are for BCM data
epicsEnvSet("NUM_CH",        "20")
# Number of samples to acquire
epicsEnvSet("NUM_SAMPLES",   "300000")
# The maximum number of time series points in the NDPluginTimeSeries plugin
epicsEnvSet("TSPOINTS",      "600000")

asynSetMinTimerPeriod(0.001)

# Uncomment the following line to set it in the IOC.
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "30000000")

# Create a Bcm driver
# BcmConfig(const char *portName, const char *devicePath,
#            int maxAddr, int numSamples, NDDataType_t dataType,
#            int maxBuffers, size_t maxMemory, int priority, int stackSize)
BcmConfig("$(PORT)", "/dev/sis8300-2", $(NUM_CH), $(NUM_SAMPLES), 7, 0, 0)
dbLoadRecords("$(SIS8300)/db/SIS8300.template",        "P=$(PREFIX),R=,           PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH0):,  PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(AICH0)")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH1):,  PORT=$(PORT),ADDR=1,TIMEOUT=1,NAME=$(AICH1)")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH2):,  PORT=$(PORT),ADDR=2,TIMEOUT=1,NAME=$(AICH2)")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH3):,  PORT=$(PORT),ADDR=3,TIMEOUT=1,NAME=$(AICH3)")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH4):,  PORT=$(PORT),ADDR=4,TIMEOUT=1,NAME=$(AICH4)")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH5):,  PORT=$(PORT),ADDR=5,TIMEOUT=1,NAME=$(AICH5)")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH6):,  PORT=$(PORT),ADDR=6,TIMEOUT=1,NAME=$(AICH6)")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH7):,  PORT=$(PORT),ADDR=7,TIMEOUT=1,NAME=$(AICH7)")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH8):,  PORT=$(PORT),ADDR=8,TIMEOUT=1,NAME=$(AICH8)")
dbLoadRecords("$(SIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH9):,  PORT=$(PORT),ADDR=9,TIMEOUT=1,NAME=$(AICH9)")

# BCM related records
dbLoadRecords("$(BCM)/db/bcmMain.template",    "P=$(PREFIX),R=,           PORT=$(PORT),ADDR=0,TIMEOUT=1")
# BCM channel related records
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM0):,   PORT=$(PORT),ADDR=10,TIMEOUT=1,NAME=$(BCM0)")
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM1):,   PORT=$(PORT),ADDR=11,TIMEOUT=1,NAME=$(BCM1)")
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM2):,   PORT=$(PORT),ADDR=12,TIMEOUT=1,NAME=$(BCM2)")
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM3):,   PORT=$(PORT),ADDR=13,TIMEOUT=1,NAME=$(BCM3)")
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM4):,   PORT=$(PORT),ADDR=14,TIMEOUT=1,NAME=$(BCM4)")
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM5):,   PORT=$(PORT),ADDR=15,TIMEOUT=1,NAME=$(BCM5)")
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM6):,   PORT=$(PORT),ADDR=16,TIMEOUT=1,NAME=$(BCM6)")
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM7):,   PORT=$(PORT),ADDR=17,TIMEOUT=1,NAME=$(BCM7)")
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM8):,   PORT=$(PORT),ADDR=18,TIMEOUT=1,NAME=$(BCM8)")
dbLoadRecords("$(BCM)/db/bcmChannel.template", "P=$(PREFIX),R=$(BCM9):,   PORT=$(PORT),ADDR=19,TIMEOUT=1,NAME=$(BCM9)")
# BCM probe related records
dbLoadRecords("$(BCM)/db/bcmProbe.template",   "P=$(PREFIX),R=$(PROBE0):, PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(PROBE0)")
dbLoadRecords("$(BCM)/db/bcmProbe.template",   "P=$(PREFIX),R=$(PROBE1):, PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(PROBE1)")
dbLoadRecords("$(BCM)/db/bcmProbe.template",   "P=$(PREFIX),R=$(PROBE2):, PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(PROBE2)")
dbLoadRecords("$(BCM)/db/bcmProbe.template",   "P=$(PREFIX),R=$(PROBE3):, PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(PROBE3)")

# Create a standard arrays plugin, set it to get data from Bcm driver.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)
# This creates a waveform large enough for 1000000x10 arrays.
#dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=10000000")
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=3000")

# Time series plugin for converted AI data
NDTimeSeriesConfigure("TS0", $(QSIZE), 0, "$(PORT)", 0, 10)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS0:,   PORT=TS0,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=,ENABLED=1")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:0:, PORT=TS0,ADDR=0,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH0)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:1:, PORT=TS0,ADDR=1,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH1)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:2:, PORT=TS0,ADDR=2,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH2)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:3:, PORT=TS0,ADDR=3,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH3)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:4:, PORT=TS0,ADDR=4,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH4)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:5:, PORT=TS0,ADDR=5,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH5)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:6:, PORT=TS0,ADDR=6,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH6)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:7:, PORT=TS0,ADDR=7,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH7)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:8:, PORT=TS0,ADDR=8,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH8)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:9:, PORT=TS0,ADDR=9,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH9)")

# Time series plugin for BCM
NDTimeSeriesConfigure("TS1", $(QSIZE), 0, "$(PORT)", 1, 10)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS1:,   PORT=TS1,ADDR=0, TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1,NCHANS=$(TSPOINTS),TIME_LINK=,ENABLED=1")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:0:, PORT=TS1,ADDR=0, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM0)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:1:, PORT=TS1,ADDR=1, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM1)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:2:, PORT=TS1,ADDR=2, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM2)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:3:, PORT=TS1,ADDR=3, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM3)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:4:, PORT=TS1,ADDR=4, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM4)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:5:, PORT=TS1,ADDR=5, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM5)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:6:, PORT=TS1,ADDR=6, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM6)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:7:, PORT=TS1,ADDR=7, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM7)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:8:, PORT=TS1,ADDR=8, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM8)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:9:, PORT=TS1,ADDR=9, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BCM9)")

# Timing MTCA EVR 300
# As per EVR MTCA 300 engineering manual ch 5.3.5
epicsEnvSet("SYS"               "EVR")
epicsEnvSet("DEVICE"            "MTCA")
epicsEnvSet("EVR_PCIDOMAIN"     "0x0")
epicsEnvSet("EVR_PCIBUS"        "0x7")
epicsEnvSet("EVR_PCIDEVICE"     "0x0")
epicsEnvSet("EVR_PCIFUNCTION"   "0x0")

#require mrfioc2,2.7.13
mrmEvrSetupPCI($(DEVICE), $(EVR_PCIDOMAIN), $(EVR_PCIBUS), $(EVR_PCIDEVICE), $(EVR_PCIFUNCTION))
dbLoadRecords("$(MRFIOC2)/db/evr-mtca-300.db", "DEVICE=$(DEVICE), SYS=$(SYS), Link-Clk-SP=88.0525")

# Trigger pulse
dbLoadRecords("$(MRFIOC2)/db/evr-softEvent.template", "DEVICE=$(DEVICE), SYS=$(SYS), EVT=14, CODE=14")
# MLVDS 0 (RearUniv32)
dbLoadRecords("$(MRFIOC2)/db/evr-pulserMap.template", "DEVICE=$(DEVICE), SYS=$(SYS), PID=0, F=Trig, ID=0, EVT=14")
# Front Panel 0 (FrontOut0)
dbLoadRecords("$(MRFIOC2)/db/evr-pulserMap.template", "DEVICE=$(DEVICE), SYS=$(SYS), PID=1, F=Trig, ID=0, EVT=14")

set_requestfile_path("$(SIS8300)/SIS8300App/Db")
set_requestfile_path("$(BCM)/bcmApp/Db")

#asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",0,255)

iocInit()


# Set some defaults for BCM
# internal clock; ~83.3 MHz
dbpf $(PREFIX)ClockSource 0
dbpf $(PREFIX)ClockDiv 3
# SMA clock; 88.0525 MHz
dbpf $(PREFIX)ClockSource 2
dbpf $(PREFIX)ClockDiv 1
# external trigger
dbpf $(PREFIX)TrigSource 1
# SIS8900 RTM
dbpf $(PREFIX)RTMType 1
#dbpf $(PREFIX)RTMTempGet 1
# Enable all channels
dbpf $(PREFIX)Enable 1

# No conversion is made for BCM data
dbpf $(PREFIX)$(AICH0):ConvFactor 1
dbpf $(PREFIX)$(AICH1):ConvFactor 1
dbpf $(PREFIX)$(AICH2):ConvFactor 1
dbpf $(PREFIX)$(AICH3):ConvFactor 1
dbpf $(PREFIX)$(AICH4):ConvFactor 1
dbpf $(PREFIX)$(AICH5):ConvFactor 1
dbpf $(PREFIX)$(AICH6):ConvFactor 1
dbpf $(PREFIX)$(AICH7):ConvFactor 1
dbpf $(PREFIX)$(AICH8):ConvFactor 1
dbpf $(PREFIX)$(AICH9):ConvFactor 1

dbpf $(PREFIX)$(AICH0):ConvOffset 0
dbpf $(PREFIX)$(AICH1):ConvOffset 0
dbpf $(PREFIX)$(AICH2):ConvOffset 0
dbpf $(PREFIX)$(AICH3):ConvOffset 0
dbpf $(PREFIX)$(AICH4):ConvOffset 0
dbpf $(PREFIX)$(AICH5):ConvOffset 0
dbpf $(PREFIX)$(AICH6):ConvOffset 0
dbpf $(PREFIX)$(AICH7):ConvOffset 0
dbpf $(PREFIX)$(AICH8):ConvOffset 0
dbpf $(PREFIX)$(AICH9):ConvOffset 0


# Disable Rear Universal Output 32
dbpf $(SYS)-$(DEVICE):RearUniv32-Ena-SP "Disabled"
# Map Rear Universal Output 32 to pulser 0
dbpf $(SYS)-$(DEVICE):RearUniv32-Src-SP 0
# Map pulser 0 to event 14
dbpf $(SYS)-$(DEVICE):Pul0-Evt-Trig0-SP 14
# Set pulser 1 width to 3100 us
dbpf $(SYS)-$(DEVICE):Pul0-Width-SP 3100
# SIS8300 is triggered
dbpf $(SYS)-$(DEVICE):RearUniv32-Ena-SP "Enabled"

# Disable Front Panel Output 0
dbpf $(SYS)-$(DEVICE):FrontOut0-Ena-SP "Disabled"
# Map Front Panel Output 0 to pulser 1
dbpf $(SYS)-$(DEVICE):FrontOut0-Src-SP 1
# Map pulser 1 to event 14
dbpf $(SYS)-$(DEVICE):Pul1-Evt-Trig0-SP 14
# Set pulser 1 width to 3100 us
dbpf $(SYS)-$(DEVICE):Pul1-Width-SP 3100
# External generator is triggered
dbpf $(SYS)-$(DEVICE):FrontOut0-Ena-SP "Enabled"


# Setup TimeSeries plugin for AI
dbpf $(PREFIX)TS0:TSNumPoints 300000
dbpf $(PREFIX)TS0:TSAcquireMode 1
dbpf $(PREFIX)TS0:TSAcquire 1
dbpf $(PREFIX)TS0:TSAveragingTime 0
# Setup TimeSeries plugin for BCM (8 times smaller than AI)
dbpf $(PREFIX)TS0:TSNumPoints 37500
dbpf $(PREFIX)TS0:TSAcquireMode 1
dbpf $(PREFIX)TS0:TSAcquire 1
dbpf $(PREFIX)TS0:TSAveragingTime 0
