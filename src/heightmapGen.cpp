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
        for(int j = y; j < (heightmap.rows - 150); j = j+4){
            heightmap.at<Vec3b>(i, j)[0] = pixelValue - 15;
            heightmap.at<Vec3b>(i, j)[1] = pixelValue - 15;
            heightmap.at<Vec3b>(i, j)[2] = pixelValue - 15;
        }
    }

    //Setter verdiene til 255 til hver fjerde pixelrad i bildet
    for(int i = x; i < ((x + sectionSize) - 5); i = i+4){
        for(int j = y; j < (heightmap.rows - 150); j++){
            heightmap.at<Vec3b>(i, j)[0] = pixelValue - 15;
            heightmap.at<Vec3b>(i, j)[1] = pixelValue - 15;
            heightmap.at<Vec3b>(i, j)[2] = pixelValue - 15;
        }
    }

    return heightmap;

}

//Lager et ulent terreng
Mat generateRandomTerrain(Mat heightmap, int x, int y, int sectionSize, int pixelValue)
{

    //Setter hvert pixel i bilde til en tilfeldig verdi
    for(int i = x; i < ((x + sectionSize) - 5); i++){
        for(int j = y; j < (heightmap.cols - 150); j++){
            int randPixelVal = 50 + rand() % 55;
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
    int pixelValue = 85;
    for(int i = 0; i < 250; i++){
        //Genererer et tall mellom 0 og heightmap.at<Vec3b>(i, j)[0] = pixelValue;
        int startPointX = x + (rand() % (((x + sectionSize)-5) - x + 1));
        int startPointY = y + rand() % (heightmap.rows - 300);
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
        for(int j = y; j < (heightmap.rows - 150); j = j+4){
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
     for(int i = x; i < ((x + sectionSize) - 5); i = i+4){
        for(int j = y; j < (heightmap.rows - 150); j++){
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
    //Lager en liste med de ulike methodene
    vector<int> methods;
    for(int i = 1; i < 6; i++){
        methods.push_back(i);
    }

    //Stokker om på metodene i vektoren
    srand(time(NULL));
    random_shuffle(methods.begin(), methods.end());

    //Lager et bilde i en størrelse som Gazebo forstår med spesifikasjoner for bilde.
    Mat flatHeightmap(513, 513, CV_8UC3, Scalar(0, 0, 0));

    for(int i = 0; i < flatHeightmap.cols; i++){
        for(int j = 0; j < flatHeightmap.rows; j++){
            flatHeightmap.at<Vec3b>(i, j)[0] = 75;
            flatHeightmap.at<Vec3b>(i, j)[1] = 75;
            flatHeightmap.at<Vec3b>(i, j)[2] = 75;
        }
    }
    
    int sectionSize = 101;
    int startPointX = 5;
    int startPointY = 150;
    int pixelValue = 90;
    Mat tmpHeightmap;
    for(int i = 0; i < methods.size(); i++){
        
        if(methods.at(i) == 1){
            tmpHeightmap = generateHorzVertTerrain(flatHeightmap, startPointX, startPointY, sectionSize);
            cout << "Metode: " << methods.at(i) << " - Ferdig " << endl;
        }else if(methods.at(i) == 2){
            tmpHeightmap = generateHorzTerrain(flatHeightmap, startPointX, startPointY, sectionSize, pixelValue);
            cout << "Metode: " << methods.at(i) << " - Ferdig " << endl;
        }else if(methods.at(i) == 3){
            tmpHeightmap = generateVertTerrain(flatHeightmap, startPointX, startPointY, sectionSize, pixelValue);
            cout << "Metode: " << methods.at(i) << " - Ferdig " << endl;
        }else if(methods.at(i) == 4){
            tmpHeightmap = generateRandomBumps(flatHeightmap, startPointX, startPointY, sectionSize);
            cout << "Metode: " << methods.at(i) << " - Ferdig " << endl;
        }else if(methods.at(i) == 5){
            tmpHeightmap = generateRandomTerrain(flatHeightmap, startPointX, startPointY, sectionSize, pixelValue);
            cout << "Metode: " << methods.at(i) << " - Ferdig " << endl;
        }else{
        cout << "Something went wrong" << endl;
        }

        startPointX = startPointX + sectionSize;
    }

    Mat heightmap = smoothImg(tmpHeightmap);

    vector<int> compression_params;
    compression_params.push_back(16);
    compression_params.push_back(9);
    imwrite("/home/hansffa/Documents/Blender/blender_terrain.png", heightmap, compression_params);
    
    imshow("heightmap", heightmap);
    waitKey(0);

    return 0;
}
