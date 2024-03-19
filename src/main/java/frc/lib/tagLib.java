// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Library to load in all of the x and y distances for the AprilTags on the field and put them into separate arrays */
public class tagLib {
    public double[] xVals = new double[9];
    public double[] yVals = new double[9];
    double[] tagIDs = new double[9];
    int currentID = 0;
    int speakerLeftTag = 1;
    int speakerRightTag = 2;
    int ampTag = 3;
    int humanPlayerLeftTag = 4;       
    int humanPlayerRightTag = 5;
    int stageRight = 6;
    int stageLeft = 7;
    int centerStage = 8;
    public tagLib(){
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.get()==DriverStation.Alliance.Red){
            tagIDs[0] = 0;
            tagIDs[speakerLeftTag] = 4;
            tagIDs[speakerRightTag] = 3;
            tagIDs[ampTag] = 5;
            tagIDs[humanPlayerLeftTag] = 9;
            tagIDs[humanPlayerRightTag] = 10;
            tagIDs[stageRight] = 11;
            tagIDs[stageLeft] = 12;
            tagIDs[centerStage] = 13;
        }
        else{
            tagIDs[0] = 0;
            tagIDs[speakerLeftTag] = 8;
            tagIDs[speakerRightTag] = 7;
            tagIDs[ampTag] = 6;
            tagIDs[humanPlayerLeftTag] = 1;
            tagIDs[humanPlayerRightTag] = 2;
            tagIDs[stageRight] = 15;
            tagIDs[stageLeft] = 16;
            tagIDs[centerStage] = 14;
        }
        xVals[speakerLeftTag] = -1.08;
        xVals[speakerRightTag] = 8.87;
        xVals[ampTag] = 0;
        xVals[humanPlayerLeftTag] = 0;
        xVals[humanPlayerRightTag] = 0;
        xVals[stageRight] = 0;
        xVals[stageLeft] = 0;
        xVals[centerStage] = 0;

        yVals[speakerLeftTag] = 10.87;
        yVals[speakerRightTag] = 9.57;
        yVals[ampTag] = 0;
        yVals[humanPlayerLeftTag] = 0;
        yVals[humanPlayerRightTag] = 0;
        yVals[stageRight] = 0;
        yVals[stageLeft] = 0;
        yVals[centerStage] = 0;
    }
    //loads in desired ID values for current tag
    public int getTagNumber(double desiredID){
        
        int currentID = 0;
        //runs until the desired tag ID is found
        //if tag isn't found, returns a value of -1, which indicates an invalid tag ID (wrong alliance)
        while(tagIDs[currentID]!=desiredID||currentID >0){
            currentID++;
            if(currentID >7){
                currentID = 0;
            }
        }
        //return array value for desired tag ID
        return currentID;
    }


    public boolean IDNotFound(){
        return currentID == 0;
    }
}