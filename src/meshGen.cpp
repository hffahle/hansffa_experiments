#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdlib.h>
#include <algorithm>

using namespace cv;
using namespace std;

//Lager vertikalt og horisontalt terreng
Mat generateHorzVertTerrain(Mat heightmap, int x, int y, int sectionSize,  int pixelValue)
{
    //Setter verdiene til 255 til hver fjerde pixelkolonne i bildet
    for(int i = x; i < (x + sectionSize); i++){
        for(int j = y; j < ((y+sectionSize)); j = j+4){
            heightmap.at<Vec3b>(i, j)[0] = pixelValue;
            heightmap.at<Vec3b>(i, j)[1] = pixelValue;
            heightmap.at<Vec3b>(i, j)[2] = pixelValue;
        }
    }

    //Setter verdiene til 255 til hver fjerde pixelrad i bildet
    for(int i = x; i < ((x + sectionSize)); i = i+4){
        for(int j = y; j < (y+sectionSize); j++){
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
    for(int i = x; i < (x + sectionSize); i++){
        for(int j = y; j < (y + sectionSize); j++){
            int randPixelVal = 75 + rand() % ((pixelValue - 75) + 1);
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
    for(int i = 0; i < 500; i++){
        //Genererer et tall mellom 0 og heightmap.at<Vec3b>(i, j)[0] = pixelValue;
        int startPointX = x + rand() % (sectionSize);
        int startPointY = y + rand() % (sectionSize);
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
    for(int i = x; i < ((x + sectionSize)); i++){
        for(int j = y; j < ((y + sectionSize) - 4); j = j+4){
            heightmap.at<Vec3b>(i, j)[0] = pixelValue;
            heightmap.at<Vec3b>(i, j)[1] = pixelValue;
            heightmap.at<Vec3b>(i, j)[2] = pixelValue;
        }
    }
    
    return heightmap;
}

//Lager horisontale dumper i terrenget
Mat generateHorzTerrain(Mat heightmap, int x, int y, int sectionSize, int pixelValue)
{
    //Setter verdiene til pixelValue til hver fjerde pixelrad i bildet
     for(int i = x; i < ((x + sectionSize) - 4); i = i+4){
        for(int j = y; j < (y + sectionSize); j++){
            heightmap.at<Vec3b>(i, j)[0] = pixelValue;
            heightmap.at<Vec3b>(i, j)[1] = pixelValue;
            heightmap.at<Vec3b>(i, j)[2] = pixelValue;
        }
    }
    return heightmap;
}

Mat smoothImg(Mat heightmap)
{
    Mat smoothMap;

    GaussianBlur(heightmap, smoothMap, Size(5, 5), 0, 0);

    return smoothMap;
}

int main()
{
    //Lager et bilde i en størrelse som Gazebo forstår med spesifikasjoner for bilde.
    Mat flatHeightmap(800, 800, CV_8UC3, Scalar(0, 0, 0));

    int sectionSize = 780;
    int startPointX = 10;
    int startPointY = 10;
    int pixelValue = 90;
    Mat tmpHeightmap;

    for(int i = 5; i < 6; i++){

        for(int i = 0; i < flatHeightmap.cols; i++){
            for(int j = 0; j < flatHeightmap.rows; j++){
                flatHeightmap.at<Vec3b>(i, j)[0] = 75;
                flatHeightmap.at<Vec3b>(i, j)[1] = 75;
                flatHeightmap.at<Vec3b>(i, j)[2] = 75;
            }
        }

        if(i == 1){
            tmpHeightmap = generateHorzVertTerrain(flatHeightmap, startPointX, startPointY, sectionSize, pixelValue);
            cout << "Metode: " << i << " - Ferdig " << endl;
            Mat horzVertHeightmap = smoothImg(tmpHeightmap);

            vector<int> compression_params;
            compression_params.push_back(16);
            compression_params.push_back(9);
            imwrite("/home/hansffa/Documents/Blender/horzVert_terrain.png", horzVertHeightmap, compression_params);

            imshow("heightmap1", horzVertHeightmap);
            waitKey(0);
        }else if(i == 2) {
            tmpHeightmap = generateHorzTerrain(flatHeightmap, startPointX, startPointY, sectionSize, pixelValue);
            cout << "Metode: " << i << " - Ferdig " << endl;
            Mat horzHeightmap = smoothImg(tmpHeightmap);

            vector<int> compression_params;
            compression_params.push_back(16);
            compression_params.push_back(9);
            imwrite("/home/hansffa/Documents/Blender/horz_terrain.png", horzHeightmap, compression_params);

            imshow("heightmap2", horzHeightmap);
            waitKey(0);
        }else if(i == 3){
            tmpHeightmap = generateVertTerrain(flatHeightmap, startPointX, startPointY, sectionSize, pixelValue);
            cout << "Metode: " << i << " - Ferdig " << endl;

            Mat vertHeightmap = smoothImg(tmpHeightmap);

            vector<int> compression_params;
            compression_params.push_back(16);
            compression_params.push_back(9);
            imwrite("/home/hansffa/Documents/Blender/vert_terrain.png", vertHeightmap, compression_params);

            imshow("heightmap3", vertHeightmap);
            waitKey(0);
        }else if(i == 4){
            tmpHeightmap = generateRandomBumps(flatHeightmap, startPointX, startPointY, sectionSize);
            cout << "Metode: " << i << " - Ferdig " << endl;

            Mat heightmap = smoothImg(tmpHeightmap);

            vector<int> compression_params;
            compression_params.push_back(16);
            compression_params.push_back(9);
            imwrite("/home/hansffa/Documents/Blender/bump_terrain.png", heightmap, compression_params);

            imshow("heightmap4", heightmap);
            waitKey(0);
        }else if(i == 5){
            tmpHeightmap = generateRandomTerrain(flatHeightmap, startPointX, startPointY, sectionSize, pixelValue);
            cout << "Metode: " << i << " - Ferdig " << endl;
            Mat heightmap = smoothImg(tmpHeightmap);

            vector<int> compression_params;
            compression_params.push_back(16);
            compression_params.push_back(9);
            imwrite("/home/hansffa/Documents/Blender/random_terrain.png", heightmap, compression_params);

            imshow("heightmap5", heightmap);
        }else{
            cout << "Something went wrong" << endl;
        }
    }

    waitKey(0);

    return 0;
}
