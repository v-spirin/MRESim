# MRESim

**A simulator for testing the behaviour of multiple robots exploring unknown environments.**

* Build MRESim:
Build with clean environment
> ant -f /home/christian/MRESim -Dnb.internal.action.name=rebuild clean jar
Build normally
> ant -f /home/christian/MRESim -Dnb.internal.action.name=build jar

* Start in GUI-Mode:
> java -jar dist/MRESim.jar
* Start in Headless Mode (Batch):
> java -cp dist/MRESim.jar batch.BatchExecution [batchfile]
The batchfile is formatet as following:
```
2 #Number of threads
sim-conf #Filename of simulation-configfile in default-directory (simconf)
team-conf #Filename of team-configfile in default-directory (teamconf)
env #Filename of the map in default-directory (environments)
sim-conf
team-conf
env
sim-conf
team-conf
env
... #Comments are NOT supported, this is NO valid example!
```
## License
This project is licensed under the terms of the GPL3 license.
The LogViewer uses HighChart ( http://www.highcharts.com/ ), which us free to use for non-profit-use, commercial- or government-use need to be licensed! As long as the LogViewer is not used, this does not affect the license of this project.
