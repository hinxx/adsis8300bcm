#!/usr/bin/python3
import re

print("Parser for DB file to generate asynPortDriver C++ elements")

DB1 = "../SIS8300bcmApp/Db/bcmMain.template"
DB2 = "../SIS8300bcmApp/Db/bcmChannel.template"
DB3 = "../SIS8300bcmApp/Db/bcmProbe.template"

def handleFile(fname):
    rec = False
    recName = None
    recAsynStr = None
    recAutoSave = False
    recDone = False
    recs = dict()
    
    f = open(fname, 'r')
    for l in f:
        #print(l)
        l = l.strip('\n')
        #print(l)
    
        if l.startswith('record'):
            rec = True
            m = re.match('record\(([a-z]+), \"(.*)\"', l)
            recType = m.group(1)
            recName = m.group(2)
        elif rec:
            if re.search('DTYP', l):
                m = re.match('\s+field\(DTYP,\s+\"(.*)\"', l)
                recDTYP = m.group(1)
            if re.search('field\(INP', l) or re.search('field\(OUT', l):
                m = re.match('\s+field\([A-Z]+,\s+\"@(.*)(.*)\"', l)
                recDir = m.group(1)
                m = re.match('([a-zA-Z]+)\((.*)\)(.*)', recDir)
                recAsyn = m.group(1)
                recAsynStr = m.group(3)
            if re.search('autosaveFields', l):
                recAutoSave = True
            if re.search('^\}$', l):
                recDone = True
        
        if rec and recDone and recAsynStr and recName:
            if recAsynStr not in recs:
                recs[recAsynStr] = []
            data = (recType, recName, recDTYP, recAsyn, recAsynStr, recAutoSave)
            #print(data)
            recs[recAsynStr].append(data)
            rec = False
            recName = None
            recAsynStr = None
            recAutoSave = False
            recDone = False
    
    f.close()
    #print(recs)
    return recs

def generateC(fname):
    print("Handling filename %s" % fname)
    recs = handleFile(fname)
    i = 0
    ss = []
    pp = []
    cc = []
    aa = []
    for k, r in sorted(recs.items()):
        r = recs[k]
        #print(r)
        i = i + len(r)
        n = r[0][4].title().replace('_', '')
        s = "#define %sString\t\t\t\"%s\"" % (n, r[0][4])
        #print(s)
        ss.append(s)
        p = "\tint m%s;" % (n)
        #print(p)
        pp.append(p)
        t = None
        if r[0][2] == 'asynInt32':
            t = 'asynParamInt32'
        elif r[0][2] == 'asynUInt32Digital':
            t = 'asynParamInt32'
        c = "createParam(%sString,\t\t%s,\t&m%s);" % (n, t, n)
        #print(c)
        cc.append(c)
        for a in r:
            if a[5]:
                aa.append(a[1])
        
    print("Have %d records" % i)
    return (ss, pp, cc, aa)

data1 = generateC(DB1)
data2 = generateC(DB2)
data3 = generateC(DB3)
# show defines
for d in data1[0]:
    print(d)
print("")
for d in data2[0]:
    print(d)
print("")
for d in data3[0]:
    print(d)

print("")
print("==============================================")
print("")

# show params
first = True
for d in data1[1]:
    print(d)
    if first:
        f = d[5:].strip(';')
        print("\t#define FIRST_BCM_PARAM %s" % (f))
        first = False
print("")
for d in data2[1]:
    print(d)
    l = d
print("")
l = None
for d in data3[1]:
    print(d)
    l = d
l = d[5:].strip(';')
print("\t#define LAST_BCM_PARAM %s" % (l))

print("")
print("==============================================")
print("")

# show creates
for d in data1[2]:
    print(d)
print("")
for d in data2[2]:
    print(d)
print("")
for d in data3[2]:
    print(d)

print("")
print("==============================================")
print("")

# show autosaves
for d in data1[3]:
    print(d)
print("")
for d in data2[3]:
    print(d)
print("")
for d in data3[3]:
    print(d)
