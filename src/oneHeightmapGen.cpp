#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdlib.h>
#include <algorithm>

using namespace cv;
using namespace std;

//Lager vertikalt og horisontalt terreng
Mat generateHorzVertTerrain(Mat heightmap, int x, int y, int sectionSize)
{
    int pixelValue = 105;
    //Setter verdiene til 255 til hver fjerde pixelkolonne i bildet
    for(int i = x; i < ((x + sectionSize) - 5); i++){
        for(int j = y; j < (heightmap.rows - 200); j = j+4){
            heightmap.at<Vec3b>(i, j)[0] = pixelValue;
            heightmap.at<Vec3b>(i, j)[1] = pixelValue;
            heightmap.at<Vec3b>(i, j)[2] = pixelValue;
        }
    }

    //Setter verdiene til 255 til hver fjerde pixelrad i bildet
    for(int i = x; i < ((x + sectionSize) - 5); i = i+4){
        for(int j = y; j < (heightmap.rows - 200); j++){
            heightmap.at<Vec3b>(i, j)[0] = pixelValue;
            heightmap.at<Vec3b>(i, j)[1] = pixelValue;
            heightmap.at<Vec3b>(i, j)[2] = pixelValue;
        }
    }

    return heightmap;

}

//Lager et ulent terreng
Mat generateRandomTerrain(Mat heightmap, int x, int y, int sectionSize, int pixelValue)
{

    //Setter hvert pixel i bilde til en tilfeldig verdi
    for(int i = x; i < ((x + sectionSize) - 5); i++){
        for(int j = y; j < (heightmap.cols - 200); j++){
            int randPixelVal = rand() % pixelValue;
            heightmap.at<Vec3b>(i,j)[0] = randPixelVal;
            heightmap.at<Vec3b>(i,j)[1] = randPixelVal;
            heightmap.at<Vec3b>(i,j)[2] = randPixelVal;
        }
    }

    return heightmap;
}

//Lager x antall dumper i terrenget
Mat generateRandomBumps(Mat heightmap, int x, int y, int sectionSize)
{
    int pixelValue = 90;
    for(int i = 0; i < 250; i++){
        //Genererer et tall mellom 0 og heightmap.at<Vec3b>(i, j)[0] = pixelValue;
        int startPointX = x + (rand() % (((x + sectionSize)-5) - x + 1));
        int startPointY = y + rand() % (heightmap.rows - 400);
        vector<int> usedpointsX;
        vector<int> usedpointsY;
        if(find(usedpointsX.begin(), usedpointsX.end(), startPointX) == usedpointsX.end() && find(usedpointsY.begin(), usedpointsY.end(), startPointY) == usedpointsY.end()){
            for(int j = startPointX; j < startPointX+3; j++){
                for(int k = startPointY; k < startPointY+3; k++){
                    heightmap.at<Vec3b>(j, k)[0] = pixelValue;
                    heightmap.at<Vec3b>(j, k)[1] = pixelValue;
                    heightmap.at<Vec3b>(j, k)[2] = pixelValue;
                }
            }
        }
        usedpointsX.push_back(startPointX);
        usedpointsY.push_back(startPointY);
    }

    return heightmap;
        
}

//Lager vertikale dumper i terrenget
Mat generateVertTerrain(Mat heightmap, int x, int y, int sectionSize, int pixelValue)
{
    //Setter verdiene til pixelValue til hver fjerde pixelkolonne i bildet
    for(int i = x; i < ((x + sectionSize) - 5); i++){
        for(int j = y; j < (heightmap.rows - 200); j = j+4){
            heightmap.at<Vec3b>(i, j)[0] = pixelValue;
            heightmap.at<Vec3b>(i, j)[1] = pixelValue;
            heightmap.at<Vec3b>(i, j)[2] = pixelValue;
        }
    }
    
    return heightmap;
}

//Lager horisontale dumper i terrenget
Mat generateHorzTerrain(Mat heightmap, int x, int y, int pixelValue)
{
    //Setter verdiene til pixelValue til hver fjerde pixelrad i bildet
    for(int i = x; i <  (heightmap.cols - 7); i = i+8){
        for(int j = y; j < (heightmap.rows - 7); j++){
            heightmap.at<Vec3b>(i, j)[0] = pixelValue;
            heightmap.at<Vec3b>(i, j)[1] = pixelValue;
            heightmap.at<Vec3b>(i, j)[2] = pixelValue;
            heightmap.at<Vec3b>(i+1, j)[0] = pixelValue;
            heightmap.at<Vec3b>(i+1, j)[1] = pixelValue;
            heightmap.at<Vec3b>(i+1, j)[2] = pixelValue;
        }
    }
    return heightmap;
}

Mat smoothImg(Mat heightmap)
{
    Mat smoothMap;

    GaussianBlur(heightmap, smoothMap, Size(7, 7), 0, 0);

    return smoothMap;
}

int main()
{
    //Lager et bilde i en størrelse som Gazebo forstår med spesifikasjoner for bilde.
    Mat flatHeightmap(1025, 1025, CV_8UC3, Scalar(0, 0, 0));

    for(int i = 0; i < flatHeightmap.cols; i++){
        for(int j = 0; j < flatHeightmap.rows; j++){
            flatHeightmap.at<Vec3b>(i, j)[0] = 75;
            flatHeightmap.at<Vec3b>(i, j)[1] = 75;
            flatHeightmap.at<Vec3b>(i, j)[2] = 75;
        }
    }
    
    int startPointX = 5;
    int startPointY = 5;
    int pixelValue = 75;
    Mat tmpHeightmap;
    for(int i = 1; i < 6; i++){
        
        /*if(i == 1){
            tmpHeightmap = generateHorzVertTerrain(flatHeightmap, startPointX, startPointY);
            cout << "Metode: " << methods.at(i) << " - Ferdig " << endl;
        }else*/ if(i == 2){
            tmpHeightmap = generateHorzTerrain(flatHeightmap, startPointX, startPointY, pixelValue);
            cout << "Metode: 2 - Ferdig " << endl;
            /*}else if(i == 3){
            tmpHeightmap = generateVertTerrain(flatHeightmap, startPointX, startPointY,  pixelValue);
            cout << "Metode: " << methods.at(i) << " - Ferdig " << endl;
        }else if(i == 4){
            tmpHeightmap = generateRandomBumps(flatHeightmap, startPointX, startPointY);
            cout << "Metode: " << methods.at(i) << " - Ferdig " << endl;
        }else if(i == 5){
            tmpHeightmap = generateRandomTerrain(flatHeightmap, startPointX, startPointY, pixelValue);
            cout << "Metode: " << methods.at(i) << " - Ferdig " << endl;*/
        }else{
            cout << "Something went wrong" << endl;
        }

    }

    Mat heightmap = smoothImg(tmpHeightmap);

    vector<int> compression_params;
    compression_params.push_back(16);
    compression_params.push_back(9);
    imwrite("/home/hansffa/.gazebo/models/heightmap/materials/textures/heightmap_test.png", heightmap, compression_params);
    
    imshow("heightmap", heightmap);
    waitKey(0);

    return 0;
}
