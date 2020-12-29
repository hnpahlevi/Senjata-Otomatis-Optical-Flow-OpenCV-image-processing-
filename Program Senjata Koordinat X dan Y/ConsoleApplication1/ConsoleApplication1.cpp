#include "stdafx.h"
#include "targetver.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <iomanip>    
#include <ctype.h>
#include <chrono>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <Windows.h>


using namespace System;
using namespace cv;
using namespace std;
#define pi 3.14159265

static void help()
{

	// print a welcome message, and the OpenCV version
	cout << "\ P R O G R A M  D E T E K S I  D A N  L A C A K  O B J E K,\n"
		"menggunakan software openCV " << CV_VERSION << endl;

}

Point2f point;				//deklarasi
bool addRemovePt = false;	//setting awal pointer, belum ditarget


static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/) //dalamn satu fungsi klik (target)

{
	if (event == EVENT_LBUTTONDOWN) //jika mouse diklik kiri 
	{
		point = Point2f((float)x, (float)y); //variabel poin akan menyimpan poin x dan y
		addRemovePt = true; //sudah ditarget
	}
}

int main(int argc, char** argv) {

	HANDLE hSerial = CreateFile(L"COM7", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if (hSerial != INVALID_HANDLE_VALUE) {
		printf("Tersambung ke Arduino \n");
		DCB dcbSerialParams;
		GetCommState(hSerial, &dcbSerialParams);
		dcbSerialParams.BaudRate = CBR_9600;
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.Parity = NOPARITY;
		dcbSerialParams.StopBits = ONESTOPBIT;
		SetCommState(hSerial, &dcbSerialParams);
	}
	else {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			printf("Serial port doesn't exist! \n");
		}
		printf("Error while setting up serial port! \n");
	}


	VideoCapture cap(0);
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);

	const int MAX_COUNT = 500;
	bool needToInit = false;



	help();
	cv::CommandLineParser parser(argc, argv, "{@input|0|}");
	string input = parser.get<string>("@input");

	if (input.size() == 1 && isdigit(input[0]))
		cap.open(input[0] - '0');
	else
		cap.open(input);

	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	namedWindow("Deteksi dan Lacak Objek", 1);
	setMouseCallback("Deteksi dan Lacak Objek", onMouse, 0); //fungsi on mouse di panggil kembali
															 //auto start = std::chrono::system_clock::now();
															 //auto time_point1 = start;
	Mat gray, prevGray, image, frame, kordinat;
	vector<Point2f> points[2];
	vector<Point2f> simpan_point1[2];


	for (;;)
	{
		cap >> frame;
		if (frame.empty())
			break;

		frame.copyTo(image);
		cvtColor(image, gray, COLOR_BGR2GRAY); //convert rgb to graylevel


		if (needToInit)
		{
			// automatic initialization
			goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04); //https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html
			cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit); //https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=cornersubpix
			addRemovePt = false;

		}
		else if (!points[0].empty())
		{

			vector<uchar> status;
			vector<float> err;
			if (prevGray.empty())
				gray.copyTo(prevGray);
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
				3, termcrit, 0, 0.001);
			size_t i, k;
			simpan_point1[1] = points[0];

			for (i = k = 0; i < points[1].size(); i++)
			{
				if (addRemovePt)
				{
					if (norm(point - points[1][i]) <= 5)
					{
						addRemovePt = false;
						continue;
					}
				}

				if (!status[i])
					continue;

				points[1][k++] = points[1][i];
				circle(image, points[1][i], 1, Scalar(255, 75, 255), -5, 8);
				cv::rectangle(image,
					cvPoint(points[1][i].x - 30 / 2, points[1][i].y - 30 / 2),
					cvPoint(points[1][i].x + 30 / 2, points[1][i].y + 30 / 2),
					CV_RGB(75, 255, 255), 1, 8
					);


			}
			points[1].resize(k);

			std::cout << "posisi objek awal (X,Y) = " << simpan_point1[1][0] << "\n";
			std::cout << "posisi objek akhir (X,Y) = " << points[1][0] << "\n";

			int x_awal = simpan_point1[1][0].x;
			int y_awal = simpan_point1[1][0].y;
			int x_akhir = points[1][0].x;
			int y_akhir = points[1][0].y;


			double kec0X;
			double perpindahanX;
			double kecepatanX;
			double percepatanX;
			double kec0Y;
			double perpindahanY;
			double kecepatanY;
			double percepatanY;

			kec0X = 0;
			kec0Y = 0;

			printf("waktu pengambilan = %.2fs\n", (double)(CLOCKS_PER_SEC * 1.5) / CLOCKS_PER_SEC);
			perpindahanX = abs(x_akhir - x_awal) / 1;
			kecepatanX = perpindahanX / ((double)(CLOCKS_PER_SEC * 1.5) / CLOCKS_PER_SEC);
			percepatanX = (kecepatanX - kec0X) / ((double)(CLOCKS_PER_SEC * 1.5) / CLOCKS_PER_SEC);
			perpindahanY = abs(y_akhir - y_awal) / 1;
			kecepatanY = perpindahanY / ((double)(CLOCKS_PER_SEC * 1.5) / CLOCKS_PER_SEC);
			percepatanY = (kecepatanY - kec0Y) / ((double)(CLOCKS_PER_SEC * 1.5) / CLOCKS_PER_SEC);

			x_awal = x_akhir;
			kec0X = kecepatanX;
			y_awal = y_akhir;
			kec0Y = kecepatanY;


			double focal_length2 = 224;
			double dot_pitch = 0.300858163;
			double Xreal2 = (x_awal)-(0.5 * 640);
			double Yreal2 = (0.5 * 480) - (y_awal);
			double tangenX = (Xreal2 / focal_length2);
			double tangenY = (Yreal2 / focal_length2);
			double sudutX = atan(tangenX) * 180 / pi;
			double sudutY = atan(tangenY) * 180 / pi;
			double sudutX2 = 35 - sudutX;
			double sudutY2 = sudutY + 52;

			/*
			char outputChars[] = "1000";
			DWORD btsIO;
			char charCheckForEscKey;

			while (true) {

			outputChars[0] = sudutX2;
			outputChars[1] = sudutY2;
			WriteFile(hSerial, outputChars, strlen(outputChars), &btsIO, NULL);
			}
			*/

			std::cout << "sudut X (rotasi) = " << sudutX << " derajat\n";
			std::cout << "sudut Y (elevasi) = " << sudutY << " derajat\n";
			std::cout << "Kecepatan X = " << kecepatanX << " pixel per second\n";
			std::cout << "Percepatan X = " << percepatanX << " pixel per second kuadrat\n";
			std::cout << "Kecepatan Y = " << kecepatanY << " pixel per second\n";
			std::cout << "Percepatan Y = " << percepatanY << " pixel per second kuadrat\n" << endl << endl;


			std::stringstream ss;
			ss << "X,Y=" << points[1][0];
			string disp1 = ss.str();
			putText(image, disp1, Point(points[1][0]), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255, 255, 75), 1.5);

			std::stringstream ss1;
			ss1 << "SudutX=" << sudutX;
			string disp2 = ss1.str();
			putText(image, disp2, Point(10, 420), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(225, 75, 255), 1.5);
			ss1 << " SudutY=" << sudutY;
			string disp3 = ss1.str();
			putText(image, disp3, Point(10, 420), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(225, 75, 255), 1.5);

			std::stringstream ss2;
			ss2 << "Vx=" << kecepatanX;
			string disp4 = ss2.str();
			putText(image, disp4, Point(10, 435), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(75, 255, 225), 1.5);
			ss2 << " Vy=" << kecepatanY;
			string disp5 = ss2.str();
			putText(image, disp5, Point(10, 435), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(75, 255, 255), 1.5);

			std::stringstream ss3;
			ss3 << "ax=" << percepatanX;
			string disp6 = ss3.str();
			putText(image, disp6, Point(10, 450), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255, 255, 75), 1.5);
			ss3 << " ay=" << percepatanY;
			string disp7 = ss3.str();
			putText(image, disp7, Point(10, 450), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255, 255, 75), 1.5);


		}

		if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
		{
			vector<Point2f> tmp;
			tmp.push_back(point);
			cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
			points[1].push_back(tmp[0]);
			addRemovePt = false;
		}


		needToInit = false;
		imshow("Deteksi dan Lacak Objek", image);


		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{

		case 'c':
			points[0].clear();
			points[1].clear();
			break;

		}

		std::swap(points[1], points[0]);
		cv::swap(prevGray, gray);
	}

	return 0;
	;
}
