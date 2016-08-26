#!/usr/bin/env python3
import sys
import datetime
from os import system
from os import listdir
from os.path import isfile, join
from subprocess import call

mypath=sys.argv[1]
onlyfiles=[]
for f in listdir(mypath):
    if(f.endswith('.sift')):
        # print(join(mypath, f))
        onlyfiles.append(join(mypath, f))

number = onlyfiles.__len__()
nowtime = datetime.datetime.now();

print("Start match at "+ str(nowtime))
for i in range(number-1):
    for j in range(number-(i+1)):
        # print(str(i)+" "+str(i+1+j))
        # print(onlyfiles[i]+' '+onlyfiles[i+1+j])
        arg=onlyfiles[i]+' '+onlyfiles[i+1+j]
        cmd='./build/MatchConsole -cuda'
        cmdstr = cmd+ " "+ arg
        cmdstr = cmdstr+'>/dev/null 2>&1'
        system(cmdstr)

nowtime = datetime.datetime.now();
print("Finished match at "+ str(nowtime))