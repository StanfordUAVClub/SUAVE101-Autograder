# -*- coding: utf-8 -*-
"""
Created on Wed Jan 28 22:33:02 2015

@author: Steven
"""

import TestDataflashLog #A very slightly modified version of the Dataflashlog module used in Mission Planner
import numpy as np
import matplotlib.pyplot as plt
import utm


#Set these as appropriate and run
plots = 0; # 0=just full route plot, 1=all leg plots too
logName =  "C:/Users/Steven/Documents/SUAVE Projects/Leadership/101 Projects/Spring2015/WEWolfman000.log";

class Route:
    
    LakeLag0 = np.matrix([572760,4142254, 38.1]);
    def __init__(self,fileName):

        self.AutoTime = 0;
        self.geo = [];
        self.WPgeo = [];
        self.WPenu = [];
        self.xTrackScore = [];
        self.timeScoreMS = [];
        self.totalDist = [];
        self.logdata = TestDataflashLog.DataflashLog()
        self.logdata.read(fileName);
        
        self.startMessage = [];
        self.endMessage = [];

        self.Legs = list();


        self.messageRange = [];
        self.ExtractWaypoints();
        self.ExtractPath();
        self.AppendWaypoints();        
        self.ExtractLegs();
        
    
    def ExtractWaypoints(self):
        
        WPKeys = list(self.logdata.channels['CMD']['CNum'].dictData.keys())
        WPKeys.sort();
        
        WPnum = 1;
        WPtot = 35;
        for k in WPKeys:
            if(self.logdata.channels['CMD']['CNum'].dictData[k]==WPnum or WPnum == WPtot):
                
                tempLine = np.matrix('0.0 0.0 0.0 0.0 0.0 0.0');
                tempLine[0,0] = self.logdata.channels['CMD']['Lat'].dictData[k];
                tempLine[0,1] = self.logdata.channels['CMD']['Lng'].dictData[k];
                tempLine[0,2] = self.logdata.channels['CMD']['Alt'].dictData[k];
                tempLine[0,3] = self.logdata.channels['CMD']['TimeMS'].dictData[k];
                
                tempLine2 = np.matrix('0.0 0.0 0.0 0.0 0.0 0.0');
                (easting,northing,_,_) = utm.from_latlon(tempLine[0,0],tempLine[0,1]);
                tempLine2[0,0] = easting - self.LakeLag0[0,0];
                tempLine2[0,1] = northing - self.LakeLag0[0,1];
                tempLine2[0,2] = tempLine[0,2];
                tempLine2[0,3] = tempLine[0,3];
                if(WPnum == 1):
                    self.WPgeo = tempLine;
                    self.WPenu = tempLine2;
                else:
                    self.WPgeo = np.append(self.WPgeo,tempLine,axis=0);
                    self.WPenu = np.append(self.WPenu,tempLine2,axis=0);
                    
                
                WPnum = WPnum + 1;
            
    def ExtractPath(self):
        PathKeys = list(self.logdata.channels['GPS']['TimeMS'].dictData.keys());
        PathKeys.sort();
        
        previousKey = 0;
        startKey = 0;
        endKey = 0;
        #A hack to help with the time not being logged correctly, considering looking at message numbers instead of time
        started = 0;
        
        for k in PathKeys:
            if(self.logdata.channels['GPS']['T'].dictData[k] < self.WPenu[0,3]):
                started = 1;
                
            if(started == 1 and self.logdata.channels['GPS']['T'].dictData[k] > self.WPenu[0,3] and startKey == 0):
                startKey = previousKey;
                tempLine = np.matrix('0.0 0.0 0.0 0.0');
                tempLine[0,0] = self.logdata.channels['GPS']['Lat'].dictData[previousKey];
                tempLine[0,1] = self.logdata.channels['GPS']['Lng'].dictData[previousKey];
                tempLine[0,2] = self.logdata.channels['GPS']['Alt'].dictData[previousKey];
                tempLine[0,3] = self.logdata.channels['GPS']['T'].dictData[previousKey];
                self.geo = tempLine;
                
                tempLine2 = np.matrix('0.0 0.0 0.0 0.0');
                (easting,northing,_,_) = utm.from_latlon(tempLine[0,0],tempLine[0,1]);
                tempLine2[0,0] = easting - self.LakeLag0[0,0];
                tempLine2[0,1] = northing - self.LakeLag0[0,1];
                tempLine2[0,2] = tempLine[0,2] - self.LakeLag0[0,2];
                tempLine2[0,3] = tempLine[0,3];
                self.enu = tempLine2;
            elif(started == 1 and self.logdata.channels['GPS']['T'].dictData[k] > self.WPenu[-1,3]+10000 and endKey == 0):
                endKey = k;
                tempLine = np.matrix('0.0 0.0 0.0 0.0');
                tempLine[0,0] = self.logdata.channels['GPS']['Lat'].dictData[k];
                tempLine[0,1] = self.logdata.channels['GPS']['Lng'].dictData[k];
                tempLine[0,2] = self.logdata.channels['GPS']['Alt'].dictData[k];
                tempLine[0,3] = self.logdata.channels['GPS']['T'].dictData[k];
                self.geo = np.append(self.geo,tempLine,axis=0);
                
                tempLine2 = np.matrix('0.0 0.0 0.0 0.0');
                (easting,northing,_,_) = utm.from_latlon(tempLine[0,0],tempLine[0,1]);
                tempLine2[0,0] = easting - self.LakeLag0[0,0];
                tempLine2[0,1] = northing - self.LakeLag0[0,1];
                tempLine2[0,2] = tempLine[0,2] - self.LakeLag0[0,2];
                tempLine2[0,3] = tempLine[0,3];
                self.enu = np.append(self.enu,tempLine2,axis=0);
            
            if(startKey != 0 and endKey == 0):
                tempLine = np.matrix('0.0 0.0 0.0 0.0');
                tempLine[0,0] = self.logdata.channels['GPS']['Lat'].dictData[k];
                tempLine[0,1] = self.logdata.channels['GPS']['Lng'].dictData[k];
                tempLine[0,2] = self.logdata.channels['GPS']['Alt'].dictData[k];
                tempLine[0,3] = self.logdata.channels['GPS']['T'].dictData[k];
                self.geo = np.append(self.geo,tempLine,axis=0);
                
                tempLine2 = np.matrix('0.0 0.0 0.0 0.0');
                (easting,northing,_,_) = utm.from_latlon(tempLine[0,0],tempLine[0,1]);
                tempLine2[0,0] = easting - self.LakeLag0[0,0];
                tempLine2[0,1] = northing - self.LakeLag0[0,1];
                tempLine2[0,2] = tempLine[0,2] - self.LakeLag0[0,2];
                tempLine2[0,3] = tempLine[0,3];
                self.enu = np.append(self.enu,tempLine2,axis=0);
            
            previousKey = k;
        
    def AppendWaypoints(self):
        leg = 1;
        startTime = self.WPenu[leg,3];
        WPeast = self.WPenu[leg-1,0];
        WPnorth = self.WPenu[leg-1,1];
        WPalt = self.WPenu[leg-1,2];
        endTime = self.WPenu[leg+1,3];
        shortestDist = 1000;
        for i in range(1,len(self.enu[:,1])):
            east = self.enu[i,0];
            north = self.enu[i,1];
            alt = self.enu[i,2];
            time = self.enu[i,3];
            
            if(time > endTime):
                leg = leg + 1;
                startTime = self.WPenu[leg,3];
                WPeast = self.WPenu[leg-1,0];
                WPnorth = self.WPenu[leg-1,1];
                WPalt = self.WPenu[leg-1,2];
                if(leg < (len(self.WPenu[:,3])-1)):
                    endTime = self.WPenu[leg+1,3];
                else:
                    endTime = self.enu[-1,3];
                shortestDist = 1000;
                
            dist = np.sqrt(np.square(WPeast-east)+np.square(WPnorth-north)+np.square(WPalt-alt));   
            
            if(dist<shortestDist):
                self.WPenu[leg-1,4] = time;
                shortestDist = dist;
                self.WPenu[leg-1,5]= dist;
            
            
            
            
            
    
    def ExtractLegs(self):
        self.timeScoreMS = 0;
        xTrackW = 0;
        length = 0;
        for i in range(1,len(self.WPenu[:,1])-1):
            startWPenu = self.WPenu[i-1,:];
            startWPenu[0,3] = self.WPenu[i-1,4]
            #startWPenu[0,3] = self.WPenu[i,3];
            endWPenu = self.WPenu[i,:];
            endWPenu[0,3] = self.WPenu[i,4];            
            #endWPenu[0,3] = self.WPenu[i+1,3];
            
            leg = Leg(startWPenu,endWPenu,self.enu);
            self.Legs.append(leg);
            self.timeScoreMS = self.timeScoreMS + leg.legTimeMS;
            xTrackW = xTrackW + leg.xTrackAvg*leg.legDist;
            length = length + leg.legDist;
        
        self.xTrackScore = xTrackW/length;
        self.totalDist = length;
        
    def PlotResults(self,plotAll):
        plt.figure
        nLegs = len(self.Legs);
        plt.figure(nLegs+1);
        plt.plot(self.WPenu[0:len(self.WPenu[:,0])-1,0],self.WPenu[0:len(self.WPenu[:,0])-1,1],marker='*');
        
        for i in range(len(self.Legs)):
            plt.figure(nLegs+1);
            plt.plot(self.Legs[i].enu[:,0],self.Legs[i].enu[:,1]);
            
            if(plotAll == 1):
                plt.figure(i+1);
                plt.plot(self.Legs[i].xTrack[:,1],self.Legs[i].xTrack[:,0]);
                plt.plot(np.matrix('0.0;0.0'),np.matrix([[0.0],[self.Legs[i].legDist]]),marker='*')            
                plt.axis('equal');
                plt.xlabel('Cross Track (m)');
                plt.ylabel('Along Track (m)');
                plt.figtext(0.6,0.86,"Leg #"+np.str(i+1));
                plt.figtext(0.6,0.83,"XTrack Area = "+np.str(np.round(self.Legs[i].xTrackTot,decimals=3))+"m^2");
                plt.figtext(0.6,0.80,"Leg Weight = "+np.str(np.round(self.Legs[i].legDist,decimals = 3))+"m");
                plt.figtext(0.6,0.77,"Leg Time = " +np.str(np.round(self.Legs[i].legTimeMS/1000.0, decimals = 3))+"s");
        plt.figure(nLegs+1);
        plt.axis('equal');
        plt.xlabel('East(m)');
        plt.ylabel('North(m)');
        plt.figtext(0.65,0.86,"Team Goose")        
        plt.figtext(0.65,0.83,"Number of Legs = "+np.str(nLegs));        
        plt.figtext(0.65,0.80,"Time Score = "+ np.str(np.round(self.timeScoreMS/1000.0,decimals=3)) + " s");
        plt.figtext(0.65,0.77,"CrossTrack Score = "+np.str(np.round(self.xTrackScore,decimals=3)));
        plt.figtext(0.65,0.74,"Total Distance = "+np.str(np.round(self.totalDist,decimals=3))+" m");
    

            
            
            
        
        
    
class Leg:
    
    def __init__(self,startWPenu, endWPenu, Penu):
        
        self.WPenu = [];
        self.legDist = [];
        self.legTimeMS = [];
        self.headingR = [];
        self.pitchR = [];
        self.type = [];
                    
        self.enu = [];
        self.xTrack = [];
        self.xTrackTot = [];
        self.xTrackAvg = [];
        self.DCM = [];
        self.deltaNED = [];
        
        
        self.WPenu = startWPenu;
        self.WPenu = np.append(self.WPenu,endWPenu,axis=0);  
        Hdistance = np.sqrt(np.square(self.WPenu[1,0]-self.WPenu[0,0])+np.square(self.WPenu[1,1]-self.WPenu[0,1]));
        if Hdistance != 0:
            self.headingR = np.arctan2(self.WPenu[1,0]-self.WPenu[0,0],self.WPenu[1,1]-self.WPenu[0,1]);
            self.pitchR = np.arctan((self.WPenu[1,2]-self.WPenu[0,2])/Hdistance);
        else:
            if self.WPenu[1,2]>self.WPenu[0,2]:
                self.headingR = 0.0;
                self.pitchR = 90*np.pi/180.0;
            else:
                self.headingR = 0.0;
                self.pitchR = -90*np.pi/180.0;
            
        self.legDist = np.sqrt(np.square(self.WPenu[1,0]-self.WPenu[0,0])+np.square(self.WPenu[1,1]-self.WPenu[0,1])+np.square(self.WPenu[1,2]-self.WPenu[0,2]));
 
        if(np.abs(self.pitchR)>89.0*np.pi/180.0):
            self.type = 2;
        else:
            self.type = 1;
        
        
        started = 0;
        for i in range(len(Penu[:,0])):
            if(started == 0):
                if(Penu[i,3] > startWPenu[0,4]):
                    times = np.array([Penu[i-1,3], Penu[i,3]]);
                    easts = np.array([Penu[i-1,0], Penu[i,0]]);
                    norths = np.array([Penu[i-1,1], Penu[i,1]]);
                    alts = np.array([Penu[i-1,2], Penu[i,2]]);
                    
                    tempLine = np.matrix('0.0,0.0,0.0,0.0');                        
                    tempLine[0,0] = np.interp(startWPenu[0,4],times,easts);
                    tempLine[0,1] = np.interp(startWPenu[0,4],times,norths);
                    tempLine[0,2] = np.interp(startWPenu[0,4],times,alts);
                    tempLine[0,3] = startWPenu[0,4];
                    
                    self.enu = tempLine;
                    started = 1;
            
            if(started ==1):
                if(Penu[i,3] > endWPenu[0,4]):
                    times = np.array([Penu[i-1,3], Penu[i,3]]);
                    easts = np.array([Penu[i-1,0], Penu[i,0]]);
                    norths = np.array([Penu[i-1,1], Penu[i,1]]);
                    alts = np.array([Penu[i-1,2], Penu[i,2]]);
                    
                    tempLine = np.matrix('0.0,0.0,0.0,0.0');                        
                    tempLine[0,0] = np.interp(endWPenu[0,4],times,easts);
                    tempLine[0,1] = np.interp(endWPenu[0,4],times,norths);
                    tempLine[0,2] = np.interp(endWPenu[0,4],times,alts);
                    tempLine[0,3] = endWPenu[0,4];
                    self.enu = np.append(self.enu,tempLine,axis=0);
                    started = 2;
                else:
                    self.enu = np.append(self.enu,Penu[i,:],axis=0);
        self.legTimeMS  = self.enu[-1,3]-self.enu[0,3];
        self.FindXTrack();
    
    def FindXTrack(self):
        
        self.xTrack = np.matrix(np.zeros((len(self.enu[:,0]),3)));
        Area = 0;
        length = 0;
        for i in range(len(self.enu[:,0])):
            self.deltaNED = np.matrix([[self.enu[i,1]-self.WPenu[0,1]],[self.enu[i,0]-self.WPenu[0,0]],[-1.0*(self.enu[i,2]-self.WPenu[0,2])]])            
            xTrack3D = self.Rotate3DVector(self.deltaNED,self.headingR,self.pitchR,0);
            self.xTrack[i,0] = xTrack3D[0,0];
            self.xTrack[i,1] = xTrack3D[1,0];
            self.xTrack[i,2] = xTrack3D[2,0];
            if(i!=0):
                if(np.sign(self.xTrack[i,1])==np.sign(self.xTrack[i-1,1])):
                    Area = Area + 0.5*(np.abs(self.xTrack[i,1])+np.abs(self.xTrack[i-1,1]))*np.abs(self.xTrack[i,0]-self.xTrack[i-1,0])
                    length = length + np.abs(self.xTrack[i,0]-self.xTrack[i-1,0]);
                else:
                    intPt = np.abs(self.xTrack[i,1])/np.abs(self.xTrack[i-1,1]);
                    Area = Area + 0.5*np.abs(self.xTrack[i-1,1])*intPt**np.abs(self.xTrack[i,0]-self.xTrack[i-1,0]);
                    Area = Area + 0.5*np.abs(self.xTrack[i,1])*(1-intPt)*np.abs(self.xTrack[i,0]-self.xTrack[i-1,0]);
                    length = length + np.abs(self.xTrack[i,0]-self.xTrack[i-1,0]);
        
        self.xTrackAvg = Area/length;
        self.xTrackTot = Area;
        
    def Rotate3DVector(self,vector, yaw, pitch, roll):
        if(yaw != 0):
            cosH = np.cos(yaw);
            sinH = np.sin(yaw);
            headingM = np.array([[cosH, sinH, 0],[-sinH, cosH, 0],[0, 0, 1]]);
        else:
            headingM = np.eye(3);
        
        if(pitch != 0):
            cosP = np.cos(pitch);
            sinP = np.sin(pitch);
            pitchM = np.array([[cosP, 0, -sinP],[0,1,0],[sinP,0,cosP]]);
        else:
            pitchM = np.eye(3);
            
        if(roll != 0):
            cosR = np.cos(roll);
            sinR = np.sin(roll);
            rollM = np.array([[1,0,0],[0,cosR,sinR],[0,-sinR,cosR]]);
        else:
            rollM = np.eye(3);
        
        self.DCM = np.dot(np.dot(rollM,pitchM),headingM);

        
        vectorOut = np.dot(self.DCM,vector);
        
        return vectorOut;
        
        
   
            
        
testing = Route(logName)

testing.PlotResults(plotAll=plots);
