
//////////////////////////////////////////////////////////////////////////////
///  [ MESH v1.0.0 ] :: MATSim Evacuation Simulation coupled with HYSPLIT  ///
//////////////////////////////////////////////////////////////////////////////

#include <iomanip>
#include <Windows.h>

#include "GL/GL.h"
#include "GL/glu.h"
#include "GL/glut.h"
#include <GL/freeglut.h>

#include <iostream>
#include <string.h>
#include <cstring>

#include <vector>
#include <algorithm>

#include "typeinfo"
#include <time.h>
#include <math.h>

//#pragma comment(linker, "/HEAP:2000000")
//#pragma comment(linker, "/STACK:2000000")

#define DIMENSION 2

#define SCREEN_WIDTH 1400 // 750
#define SCREEN_HEIGHT 900 // 800

#define SIZING_PARAMETER 0.023 // 0.13
#define PARALLEL_PARAMETER_X (-14109300-26087)*SIZING_PARAMETER // -14129450*SIZING_PARAMETER(snu), -14109300*SIZING_PARAMETER(seoul)
#define PARALLEL_PARAMETER_Y (-4497500-19565)*SIZING_PARAMETER // -4501350*SIZING_PARAMETER(snu), -4497500*SIZING_PARAMETER(seoul)

#define AGENT_SIZER 4

#define AGENT_NUMBER 1000

#define NUM_OF_NODES 99143 // 1668(snu), 99143(seoul)
#define NUM_OF_LINKS 267493 // 4419(snu), 267493(seoul)

#define NUM_OF_EVENT 51523 // 11170(snu), 32549(500), 51523(1k), 135186(2k), 313654(5k), 3301055(50k)

#define SIMULATION_START_TIME 202009071200 // (yyyymmddhhmm)
#define SIMULATION_STEP 360 // min
#define INITIAL_TIME 28800

#define NODE_ITEMS 2
#define LINK_ITEMS 4
#define EVENT_ITEMS 3

#define REWRITE_NETWORK 0
#define REWRITE_EVENT 0

#define CHECK_NODES 0
#define CHECK_LINKS 0
#define CHECK_EVENT 0
#define CHECK_LINKPATH 0
#define CHECK_NODEPATH 0
#define CHECK_PATHTIME 0
#define CHECK_G_TIME 0
#define CHECK_VELOCITY 0

#define CHECK_NODENAME 0
#define CHECK_LINKNAME 0

#define CHECK_NODE_POSITION 0

#define CHECK_TOTAL_DOSE_OF_AGENT 0
#define CHECK_SHELTER_DOSE 0
#define CHECK_BYTIME 0

#define COLORING_TRAFFIC 0
#define COLORING_DOSE 1

#define CHASE 0 // CHASING AN AGENT ( 0 == ALL )

#define HYSPLIT_REALTIME 0

void display();
void reshape(int, int);
void timer(int);

void network();
void event();

void open_network();
void open_event();

void Keys(int, int, int);
//void mouse(int, int);
void MouseWheel(int, int, int, int);

void myMotion(int x, int y);
void myMouse(int button, int state, int x, int y);
void saveMousePos(int x, int y);

int winw = 1, winh = 1;

double cam_xmin, cam_xmax, cam_ymin, cam_ymax;
double cam_mousex, cam_mousey;
bool dragging;

double zoom = 1.0;

int nodeIdx(char* arr);
int linkIdx(char* arr);

int HYSPLIT_STEP = 1;
int HYSPLIT_TICK = 0;

double G_TIME = INITIAL_TIME; // 36000 // start time (msec)

int FPS = 60; // frame per second
float SPEED_PARAMETER = 0.15; // simulation speed
double TIME_STEP = 1000 / FPS; // default

int maxID;

int yr, date, hr, min;

int bottomside = -SCREEN_HEIGHT / 2;
int upside = SCREEN_HEIGHT / 2;
int leftside = -SCREEN_WIDTH / 2;
int rightside = SCREEN_WIDTH / 2;

double nodeArray[NUM_OF_NODES][NODE_ITEMS]; // nodeArray: (x, y)
float linkArray[NUM_OF_LINKS][LINK_ITEMS]; // linkArray: (from node ID, to node ID, length, freespeed)
int eventArray[NUM_OF_EVENT][EVENT_ITEMS]; // eventArray: (vehicle ID, link ID, at time)

char nodename[NUM_OF_NODES][128]; // string ID of nodes
char linkname[NUM_OF_LINKS][128]; // string ID of links

int NON = 0; // number of nodes
int NOL = 0; // number of links

int ViewX = 0;
int ViewY = 0;

float color_orange = 0.001;
float color_red = 5.0;

//int drawlinks = 0;

double x_position[AGENT_NUMBER] =
{ 50000 * SIZING_PARAMETER + PARALLEL_PARAMETER_X, }; // Initial x_position
double y_position[AGENT_NUMBER] =
{ 50000 * SIZING_PARAMETER + PARALLEL_PARAMETER_Y, }; // Initial y_position

float velocity[AGENT_NUMBER] =
{ 0, };
float agentdose[AGENT_NUMBER] =
{ 0, };

float conc[SCREEN_WIDTH + 1][SCREEN_HEIGHT + 1] =
{ 0, };


// agentpath_link[vehicle ID]: (link ID, link ID, link ID, ...)
std::vector<std::vector<int>> agentpath_link;

// agentpath_node[vehicle ID]: (node ID, node ID, node ID, ...)
std::vector<std::vector<int>> agentpath_node;

// pathtime[vehicle ID]: (at time, at time, at time, ...)
std::vector<std::vector<int>> pathtime;

std::vector<float> totaldosebytime;
std::vector<int> orangebytime;
std::vector<int> redbytime;


char* ftoa(double f) {

	char buf[256] = {};
	sprintf(buf, "%f", f);

	return buf;
}

int nodeIdx(char* arr) {

	int num = 0;

	while (1) {

		if (strcmp(nodename[num], arr) == 0)
			return num;
		else
			num++;
	}
}


int linkIdx(char* arr) {

	int num = 0;

	while (1) {

		if (strcmp(linkname[num], arr) == 0)
			return num;
		else
			num++;
	}
}


void network() {

	char FileName[128];
	char inputString[1000];
	char* item;

	char output[256];
	sprintf(output, "./log/nodes.txt");
	FILE* outFile;
	outFile = fopen(output, "w");

	sprintf(FileName, "./input_%d/network.xml", AGENT_NUMBER);
	//strcpy(FileName, "./input/network.xml");
	FILE* fd;
	fd = fopen(FileName, "r");

	int end;
	end = fscanf(fd, "%s", &inputString);

	// READ NODE DATA //

	while (strcmp(inputString, "<nodes>") != 0 && (end != -1)) end = fscanf(fd, "%s", &inputString);

	int i = 0;

	if (CHECK_NODES) std::cout << "// Check node data //" << std::endl << std::endl;


	clock_t start1, end1;

	start1 = clock();


	while (1) {

		end = fscanf(fd, "%s", &inputString);

		if (strcmp(inputString, "</nodes>") == 0) break;

		//nodeArray[i][1] = double(i+1);
		//if (CHECK_NODES) std::cout << std::setfill(' ') << std::setw(7) << nodeArray[i][0] << " ";

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		//printf("%s\n",item);
		fprintf(outFile, "%s ", item);
		strcpy(nodename[i], item);
		if (CHECK_NODES) std::cout << std::setfill(' ') << std::setw(7) << item << " ";

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		fprintf(outFile, "%f ", atof(item));
		nodeArray[i][0] = atof(item);
		if (CHECK_NODES) std::cout << std::setfill(' ') << std::setw(7) << item << " ";

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		fprintf(outFile, "%f\n", atof(item));
		nodeArray[i][1] = atof(item);
		if (CHECK_NODES) std::cout << std::setfill(' ') << std::setw(7) << item << " ";

		if (CHECK_NODES) std::cout << std::endl;

		end = fscanf(fd, "%s", &inputString);
		end = fscanf(fd, "%s", &inputString);

		i++;
	}

	NON = i;

	//if (CHECK_NODES) std::cout << std::endl << "< total num of nodes: " << i << " >" << std::endl;

	std::cout << std::endl << "< total num of nodes: " << i << " >" << std::endl << std::endl;

	//std::cout << std::endl << "< size of nodes: " << std::size(nodeArray) << " >" << std::endl;
	fclose(outFile);

	end1 = clock();

	std::cout << "calculating time for reading <nodes>: " << (double)end1 - start1 << std::endl << std::endl;
	// READ LINK DATA //

	start1 = clock();

	sprintf(output, "./log/links.txt");
	outFile = fopen(output, "w");

	while (strcmp(inputString, "<links") != 0 && (end != -1))
		end = fscanf(fd, "%s", &inputString);

	end = fscanf(fd, "%s", &inputString); // skip capperiod
	end = fscanf(fd, "%s", &inputString); // skip effective cell size
	end = fscanf(fd, "%s", &inputString); // skip effective lane width

	i = 0;

	int Idx = 0;

	if (CHECK_LINKS) std::cout << std::endl << std::endl
		<< "// Check link data //" << std::endl << std::endl;

	while (1) {

		end = fscanf(fd, "%s", &inputString);
		//printf("%s\n", inputString);
		if (strcmp(inputString, "<!--") == 0) break;
		if (strcmp(inputString, "</links>") == 0) break;

		//linkArray[i][1] = int(i + 1);
		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << i + 1 << " ";

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		fprintf(outFile, "%s ", item);
		strcpy(linkname[i], item);
		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << item << " ";

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		//linkArray[i][1] = 1;

		Idx = nodeIdx(item) + 1;
		fprintf(outFile, "%d ", Idx);
		linkArray[i][0] = Idx;
		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << linkArray[i][0] << " ";

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		//linkArray[i][2] = 1;

		Idx = nodeIdx(item) + 1;
		fprintf(outFile, "%d ", Idx);
		linkArray[i][1] = Idx;
		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << linkArray[i][1] << " ";

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		fprintf(outFile, "%f ", atof(item));
		linkArray[i][2] = atof(item);
		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << linkArray[i][2] << " ";

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		fprintf(outFile, "%f\n", atof(item));
		linkArray[i][3] = atof(item);
		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << linkArray[i][3] << " ";


		// skip capacity, permlanes, oneway, modes
		end = fscanf(fd, "%s", &inputString);
		end = fscanf(fd, "%s", &inputString);
		end = fscanf(fd, "%s", &inputString);

		end = fscanf(fd, "%s", &inputString);
		end = fscanf(fd, "%s", &inputString);
		end = fscanf(fd, "%s", &inputString);

		if (CHECK_LINKS) std::cout << std::endl;
		i++;
	}
	NOL = i;
	//if (CHECK_LINKS) std::cout << std::endl 
	//	<< "< total num of links: " << i << " >" << std::endl;
	std::cout << std::endl
		<< "< total num of links: " << i << " >" << std::endl << std::endl;

	fclose(outFile);

	end1 = clock();

	std::cout << "calculating time for reading <links>: " << (double)end1 - start1 << std::endl << std::endl;

	fclose(fd);

}

void event() {

	char FileName[128];
	char inputString[1000];
	char* item;

	clock_t start2, end2;
	double result;

	start2 = clock();

	char output[256];
	sprintf(output, "./log/events_%d.txt", AGENT_NUMBER);
	FILE* outFile;
	outFile = fopen(output, "w");

	sprintf(FileName, "./input_%d/events.xml", AGENT_NUMBER);
	FILE* fd;
	fd = fopen(FileName, "r");

	int end;
	end = fscanf(fd, "%s", &inputString);

	// READ EVENT DATA //

	int i = 0;
	int tmp = 0;

	if (CHECK_EVENT) std::cout << std::endl << std::endl
		<< "// Check event data //" << std::endl << std::endl;

	while (1) {

		while (strcmp(inputString, "<event") != 0 && (end != -1))
			end = fscanf(fd, "%s", &inputString);

		if (end == -1) break;

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		tmp = atoi(item);
		//if (CHECK_EVENT) std::cout << item << " ";
		//std::cout << item << " ";

		end = fscanf(fd, "%s", &inputString);
		item = strtok(inputString, "\"");
		item = strtok(NULL, "\"");
		//if (CHECK_EVENT) std::cout << item << " ";

		if (strcmp(item, "entered") == 0)
		{
			end = fscanf(fd, "%s", &inputString);

			fprintf(outFile, "%d ", tmp);
			eventArray[i][2] = tmp;
			if (CHECK_EVENT) std::cout << "time = " << eventArray[i][2] << ", ";

			end = fscanf(fd, "%s", &inputString);
			item = strtok(inputString, "\"");
			item = strtok(NULL, "\"");

			//eventArray[i][1] = 1;

			tmp = linkIdx(item) + 1;
			fprintf(outFile, "%d ", tmp);
			eventArray[i][1] = tmp;
			if (CHECK_EVENT) std::cout << "link = " << eventArray[i][1] << ", ";

			end = fscanf(fd, "%s", &inputString);
			item = strtok(inputString, "\"");
			item = strtok(NULL, "\"");

			fprintf(outFile, "%d\n", atoi(item) - 1);
			eventArray[i][0] = atoi(item) - 1;
			if (CHECK_EVENT) std::cout << "vehicle = " << eventArray[i][0] << std::endl;

			i++;
		}
	}
	//if (CHECK_EVENT) std::cout << std::endl 
	//	<< "< total num of event: " << i << " >" << std::endl;
	std::cout << std::endl
		<< "< total num of event: " << i << " >" << std::endl;

	fclose(outFile);

	maxID = 0;

	for (int j = 0; j < i; j++)
	{
		if (eventArray[j][0] >= maxID) maxID = eventArray[j][0];
	}

	std::cout << std::endl;
	std::cout << "Agent number = " << maxID + 1 << std::endl << std::endl;

	agentpath_node.resize(maxID + 1, std::vector<int>(1));
	agentpath_link.resize(maxID + 1, std::vector<int>(1));
	pathtime.resize(maxID + 1, std::vector<int>(1));

	//for (std::vector<double> vec : pathtime) {
	//	for (int n : vec) {
	//		printf("%d ", n);
	//	}
	//	std::cout << std::endl;
	//}

	for (int a = 0; a < i; a++)
	{

		if (pathtime[eventArray[a][0]].size() == 1 && pathtime[eventArray[a][0]][0] == 0)
			pathtime[eventArray[a][0]].clear();
		if (agentpath_node[eventArray[a][0]].size() == 1 && agentpath_node[eventArray[a][0]][0] == 0)
			agentpath_node[eventArray[a][0]].clear();
		if (agentpath_link[eventArray[a][0]].size() == 1 && agentpath_link[eventArray[a][0]][0] == 0)
			agentpath_link[eventArray[a][0]].clear();


		int n = 0;

		if (pathtime[eventArray[a][0]].size() == 0)
		{
			pathtime[eventArray[a][0]].push_back(eventArray[a][2]);
			agentpath_node[eventArray[a][0]].push_back((int)eventArray[a][1]);
			agentpath_link[eventArray[a][0]].push_back((int)eventArray[a][1]);
		}
		else {
			n = 0;

			while (1)
			{

				if (n == pathtime[eventArray[a][0]].size()) break;

				if (pathtime[eventArray[a][0]][n] < eventArray[a][2]) n++;
				else break;

			}

			pathtime[eventArray[a][0]].insert(pathtime[eventArray[a][0]].begin() + n, eventArray[a][2]);
			agentpath_node[eventArray[a][0]].insert(agentpath_node[eventArray[a][0]].begin() + n, (int)eventArray[a][1]);
			agentpath_link[eventArray[a][0]].insert(agentpath_link[eventArray[a][0]].begin() + n, (int)eventArray[a][1]);
		}

	}

	int linenum = 1;

	sprintf(output, "./log/pathtime_%d.txt", AGENT_NUMBER);
	outFile = fopen(output, "w");

	// PRINT PATHTIME ARRAY



	if (CHECK_PATHTIME) std::cout << std::endl << std::endl << "// Check path time //" << std::endl << std::endl;

	for (std::vector<int> vec : pathtime) {

		if (CHECK_PATHTIME) std::cout << "ID: " << std::setfill(' ') << std::setw(4) << linenum << " //  ";

		for (int n : vec) {
			fprintf(outFile, "%d ", n);
			if (CHECK_PATHTIME) std::cout << std::setfill(' ') << std::setw(4) << n << " ";
		}

		fprintf(outFile, "\n");
		if (CHECK_PATHTIME) std::cout << std::endl;
		linenum++;
	}


	fclose(outFile);

	sprintf(output, "./log/linkpath_%d.txt", AGENT_NUMBER);
	outFile = fopen(output, "w");


	// PRINT AGENT PATH ARRAY BY LINK ID

	linenum = 1;


	if (CHECK_LINKPATH) std::cout << std::endl << std::endl << "// Check link path //" << std::endl << std::endl;

	for (std::vector<int> vec : agentpath_link) {

		if (CHECK_LINKPATH) std::cout << "ID: " << std::setfill(' ') << std::setw(4) << linenum << " //  ";

		for (int n : vec) {
			fprintf(outFile, "%d ", n);
			if (CHECK_LINKPATH) std::cout << std::setfill(' ') << std::setw(4) << n << " ";
		}

		fprintf(outFile, "\n");
		if (CHECK_LINKPATH) std::cout << std::endl;
		linenum++;
	}


	fclose(outFile);

	sprintf(output, "./log/nodepath_%d.txt", AGENT_NUMBER);
	outFile = fopen(output, "w");

	// PRINT AGENT PATH ARRAY BY NODE ID


	if (CHECK_NODEPATH) std::cout << std::endl
		<< std::endl << "// Check node path //" << std::endl << std::endl;

	for (int num = 0; num < maxID + 1; num++) {

		if (CHECK_NODEPATH) std::cout << "ID: "
			<< std::setfill(' ') << std::setw(4) << num + 1 << " //  ";

		//if (num==8) std::cout << linkArray[agentpath_node[num][0]][0] << " / "
		//	<< linkArray[agentpath_node[num][0]][1] << " / "
		//	<< linkArray[agentpath_node[num][0]][2] << " / "
		//	<< linkArray[agentpath_node[num][0]][3] << std::endl;

		agentpath_node[num].insert(agentpath_node[num].begin(), (int)linkArray[agentpath_node[num][0] - 1][0]);

		fprintf(outFile, "%d ", agentpath_node[num][0]);
		if (CHECK_NODEPATH) std::cout
			<< std::setfill(' ') << std::setw(4) << agentpath_node[num][0] << " ";

		for (int n = 0; n < agentpath_node[num].size() - 1; n++) {

			agentpath_node[num][n + 1] = (int)linkArray[agentpath_node[num][n + 1] - 1][1];
			fprintf(outFile, "%d ", agentpath_node[num][n + 1]);
			if (CHECK_NODEPATH) std::cout
				<< std::setfill(' ') << std::setw(4) << agentpath_node[num][n + 1] << " ";

		}

		fprintf(outFile, "\n");
		if (CHECK_NODEPATH) std::cout << std::endl;
	}

	end2 = clock();

	std::cout << "calculating time for read <event.xml>: " << (float)(end2 - start2) << std::endl << std::endl;

	fclose(outFile);
	fclose(fd);

}

void open_network() {

	char FileName[128];
	char inputString[1000];

	sprintf(FileName, "./log/nodes.txt");
	FILE* fd;
	fd = fopen(FileName, "r");

	int end;
	int i = 0;

	if (CHECK_NODES) std::cout << "// Check node data //" << std::endl << std::endl;

	while (1) {

		end = fscanf(fd, "%s", &inputString);
		if (end == -1) break;
		strcpy(nodename[i], inputString);

		if (CHECK_NODES) std::cout << std::setfill(' ') << std::setw(7) << inputString << " ";

		end = fscanf(fd, "%s", &inputString);
		nodeArray[i][0] = atof(inputString);

		if (CHECK_NODES) std::cout << std::setfill(' ') << std::setw(7) << inputString << " ";

		end = fscanf(fd, "%s", &inputString);
		nodeArray[i][1] = atof(inputString);

		if (CHECK_NODES) std::cout << std::setfill(' ') << std::setw(7) << inputString << " " << std::endl;

		i++;
	}

	fclose(fd);

	NON = i;

	std::cout << std::endl << "< total num of nodes: " << i << " >" << std::endl << std::endl;

	// READ LINK DATA //

	strcpy(FileName, "./log/links.txt");
	fd = fopen(FileName, "r");

	i = 0;

	while (1) {

		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << i + 1 << " ";

		end = fscanf(fd, "%s", &inputString);
		if (end == -1) break;
		strcpy(linkname[i], inputString);

		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << inputString << " ";

		end = fscanf(fd, "%s", &inputString);
		linkArray[i][0] = atoi(inputString);

		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << linkArray[i][0] << " ";

		end = fscanf(fd, "%s", &inputString);
		linkArray[i][1] = atoi(inputString);

		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << linkArray[i][1] << " ";

		end = fscanf(fd, "%s", &inputString);
		linkArray[i][2] = atof(inputString);

		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << linkArray[i][2] << " ";

		end = fscanf(fd, "%s", &inputString);
		linkArray[i][3] = atof(inputString);

		if (CHECK_LINKS) std::cout << std::setfill(' ') << std::setw(15) << linkArray[i][3] << " " << std::endl;

		i++;
	}

	NOL = i;

	std::cout << std::endl
		<< "< total num of links: " << i << " >" << std::endl << std::endl;

	fclose(fd);

}

void open_event() {

	char FileName[128];
	char inputString[1000];
	char* item;

	sprintf(FileName, "./log/events_%d.txt", AGENT_NUMBER);
	FILE* fd;
	fd = fopen(FileName, "r");

	int end;

	int i = 0;

	if (CHECK_EVENT) std::cout << std::endl << std::endl
		<< "// Check event data //" << std::endl << std::endl;

	while (1) {

		end = fscanf(fd, "%s", &inputString);
		if (end == -1) break;
		eventArray[i][2] = atoi(inputString);

		if (CHECK_EVENT) std::cout << "time = " << eventArray[i][2] << ", ";

		end = fscanf(fd, "%s", &inputString);
		eventArray[i][1] = atoi(inputString);

		if (CHECK_EVENT) std::cout << "link = " << eventArray[i][1] << ", ";

		end = fscanf(fd, "%s", &inputString);
		eventArray[i][0] = atoi(inputString);

		if (CHECK_EVENT) std::cout << "vehicle = " << eventArray[i][0] << std::endl;

		i++;

	}

	std::cout << std::endl
		<< "< total num of event: " << i << " >" << std::endl;

	fclose(fd);

	maxID = 0;

	for (int j = 0; j < i; j++)
	{
		if (eventArray[j][0] >= maxID) maxID = eventArray[j][0];
	}

	std::cout << std::endl;
	std::cout << "Agent number = " << maxID + 1 << std::endl << std::endl;

	agentpath_node.resize(maxID + 1, std::vector<int>(1));
	agentpath_link.resize(maxID + 1, std::vector<int>(1));
	pathtime.resize(maxID + 1, std::vector<int>(1));

	//for (std::vector<double> vec : pathtime) {
	//	for (int n : vec) {
	//		printf("%d ", n);
	//	}
	//	std::cout << std::endl;
	//}

	for (int a = 0; a < i; a++)
	{

		if (pathtime[eventArray[a][0]].size() == 1 && pathtime[eventArray[a][0]][0] == 0)
			pathtime[eventArray[a][0]].clear();
		if (agentpath_node[eventArray[a][0]].size() == 1 && agentpath_node[eventArray[a][0]][0] == 0)
			agentpath_node[eventArray[a][0]].clear();
		if (agentpath_link[eventArray[a][0]].size() == 1 && agentpath_link[eventArray[a][0]][0] == 0)
			agentpath_link[eventArray[a][0]].clear();


		int n = 0;

		if (pathtime[eventArray[a][0]].size() == 0)
		{
			pathtime[eventArray[a][0]].push_back(eventArray[a][2]);
			agentpath_node[eventArray[a][0]].push_back((int)eventArray[a][1]);
			agentpath_link[eventArray[a][0]].push_back((int)eventArray[a][1]);
		}
		else {
			n = 0;

			while (1)
			{

				if (n == pathtime[eventArray[a][0]].size()) break;

				if (pathtime[eventArray[a][0]][n] < eventArray[a][2]) n++;
				else break;

			}

			pathtime[eventArray[a][0]].insert(pathtime[eventArray[a][0]].begin() + n, eventArray[a][2]);
			agentpath_node[eventArray[a][0]].insert(agentpath_node[eventArray[a][0]].begin() + n, (int)eventArray[a][1]);
			agentpath_link[eventArray[a][0]].insert(agentpath_link[eventArray[a][0]].begin() + n, (int)eventArray[a][1]);
		}

	}

	int linenum = 1;


	if (CHECK_PATHTIME) {

		std::cout << std::endl << std::endl << "// Check path time //" << std::endl << std::endl;

		for (std::vector<int> vec : pathtime) {

			std::cout << "ID: " << std::setfill(' ') << std::setw(4) << linenum << " //  ";

			for (int n : vec) {
				std::cout << std::setfill(' ') << std::setw(4) << n << " ";
			}

			std::cout << std::endl;
			linenum++;
		}
	}

	linenum = 1;

	if (CHECK_LINKPATH) {

		std::cout << std::endl << std::endl << "// Check link path //" << std::endl << std::endl;

		for (std::vector<int> vec : agentpath_link) {

			std::cout << "ID: " << std::setfill(' ') << std::setw(4) << linenum << " //  ";

			for (int n : vec) {
				std::cout << std::setfill(' ') << std::setw(4) << n << " ";
			}

			std::cout << std::endl;
			linenum++;
		}
	}

	// PRINT AGENT PATH ARRAY BY NODE ID

	if (CHECK_NODEPATH) std::cout << std::endl
		<< std::endl << "// Check node path //" << std::endl << std::endl;

	for (int num = 0; num < maxID + 1; num++) {

		if (CHECK_NODEPATH) std::cout << "ID: "
			<< std::setfill(' ') << std::setw(4) << num + 1 << " //  ";

		agentpath_node[num].insert(agentpath_node[num].begin(), (int)linkArray[agentpath_node[num][0] - 1][0]);

		if (CHECK_NODEPATH) std::cout
			<< std::setfill(' ') << std::setw(4) << agentpath_node[num][0] << " ";

		for (int n = 0; n < agentpath_node[num].size() - 1; n++) {

			agentpath_node[num][n + 1] = (int)linkArray[agentpath_node[num][n + 1] - 1][1];

			if (CHECK_NODEPATH) std::cout
				<< std::setfill(' ') << std::setw(4) << agentpath_node[num][n + 1] << " ";

		}

		if (CHECK_NODEPATH) std::cout << std::endl;
	}

	fclose(fd);

}


void init()
{
	glClearColor(0.0, 0.0, 0.0, 1.0);
	cam_mousex = 0.;
	cam_mousey = 0.;
	dragging = false;
}


void display()
{
	//int X = (ViewX - (WIDTH / 2));
	//int Y = -(ViewY - (HEIGHT / 2));

	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	//glEnable(GL_DEPTH_TEST);

	//glRotatef(rotate_x, 1.0, 0.0, 0.0);
	//glRotatef(rotate_y, 0.0, 1.0, 0.0);

	glScalef(zoom, zoom, 1.0f);

	//for (int a = 0; a < 2000; a++) {
	//	glBegin(GL_POLYGON);
	//	glColor3f(1.0, 1.0, 1.0);
	//	glVertex2f(a/5, 0);
	//	glVertex2f(a/5, 2*a/5);
	//	glVertex2f((a+1)/5, 2*a/5);
	//	glVertex2f((a+1)/5, 0);
	//	glEnd();
	//}

	//float gridcenter_x = 0;
	//float gridcenter_y = 0;

	//for (int lat = 0; lat < 420; lat++) {
	//	for (int lon = 0; lon < 270; lon++) {

	//		gridcenter_x = -700 + lat / 0.3;
	//		gridcenter_y = -450 + lon / 0.3;

	//		glBegin(GL_QUADS);
	//		glColor3f(0.5, 0.5, 0.0);
	//		glVertex2f(gridcenter_x - 10/3, gridcenter_y - 10/3);
	//		glVertex2f(gridcenter_x - 10/3, gridcenter_y + 10/3);
	//		glVertex2f(gridcenter_x + 10/3, gridcenter_y - 10/3);
	//		glVertex2f(gridcenter_x + 10/3, gridcenter_y + 10/3);
	//		glEnd();

	//	}
	//}


	//float gridcenter_x = -700;
	//float gridcenter_y = -450;

	//for (int lat = 0; lat < 700; lat++) {
	//	for (int lon = 0; lon < 450; lon++) {

	//		gridcenter_x = -700 + lat * 2;
	//		gridcenter_y = -450 + lon * 2;

	//		glBegin(GL_QUADS);
	//		glColor3f(0.5, 0.5, 0.0);
	//		glVertex2f(gridcenter_x, gridcenter_y);
	//		glVertex2f(gridcenter_x, gridcenter_y + 2);
	//		glVertex2f(gridcenter_x + 2, gridcenter_y + 2);
	//		glVertex2f(gridcenter_x + 2, gridcenter_y);
	//		glEnd();

	//	}
	//}



	//glBegin(GL_QUADS);
	//glColor3f(1.0, (lat + lon) / 690, 1.0);
	//glVertex2f(gridcenter_x - 10 / 3, gridcenter_y - 10 / 3);
	//glVertex2f(gridcenter_x - 10 / 3, gridcenter_y + 10 / 3);
	//glVertex2f(gridcenter_x + 10 / 3, gridcenter_y - 10 / 3);
	//glVertex2f(gridcenter_x + 10 / 3, gridcenter_y + 10 / 3);
	//glEnd();


	//gluOrtho2D(leftside, rightside, bottomside, upside);
	//glutPostRedisplay();

	//std::cout << 1 << std::endl;


	// DISPLAY LINKS

	//if (drawlinks == 0) {
	for (int i = 0; i < NUM_OF_LINKS; i++)
	{
		glBegin(GL_LINES);
		glLineWidth(0.05);
		glColor3f(0.5, 0.5, 0.5);

		glVertex2f(nodeArray[(int)linkArray[i][0] - 1][0], nodeArray[(int)linkArray[i][0] - 1][1]);
		glVertex2f(nodeArray[(int)linkArray[i][1] - 1][0], nodeArray[(int)linkArray[i][1] - 1][1]);

		glEnd();
		//drawlinks = 1;
	}
	//}

	for (int mm = 0; mm < SCREEN_WIDTH + 1; mm++)
		for (int nn = 0; nn < SCREEN_HEIGHT + 1; nn++)
			conc[mm][nn] = 0;

	char input[256];

	// hysplit data 016(0116), 251(0907), 275(1001) //

	hr = (SIMULATION_START_TIME / 100) % 100 + HYSPLIT_STEP / 60;
	min = SIMULATION_START_TIME % 100 + HYSPLIT_STEP % 60;

	if (HYSPLIT_REALTIME)
		sprintf(input, "./hysplit_0116/ABMgrid_016_%02d%02d", hr + 1, min);
	else
		sprintf(input, "./hysplit_0116/ABMgrid_016_%02d%02d", hr, min);

	//std::cout << (SIMULATION_START_TIME / 100) % 100 + HYSPLIT_STEP / 60 << "  " << SIMULATION_START_TIME % 100 + HYSPLIT_STEP % 60 << std::endl;

	//if (HYSPLIT_REALTIME)
	//	sprintf(input, "./hysplit_0116/ABMgrid_016_%02d%02d",
	//		(SIMULATION_START_TIME / 100) % 100 + 1 + HYSPLIT_STEP / 60, SIMULATION_START_TIME % 100 + HYSPLIT_STEP % 60);
	//else
	//	sprintf(input, "./hysplit_0116/ABMgrid_016_%02d%02d",
	//		(SIMULATION_START_TIME / 100) % 100 + HYSPLIT_STEP / 60, SIMULATION_START_TIME % 100 + HYSPLIT_STEP % 60);

	FILE* inFile;

	inFile = fopen(input, "r");

	float centergrid_x;
	float centergrid_y;
	float concentration;
	int end;

	int m, n;

	while (1) {

		end = fscanf(inFile, "%f", &centergrid_y);
		if (end == -1) break;
		end = fscanf(inFile, "%f", &centergrid_x);
		end = fscanf(inFile, "%f", &concentration);

		centergrid_x = (centergrid_x - 126.9743) * 10000 / 3;
		centergrid_y = (centergrid_y - 37.5635) * 10000 / 3;

		m = floor(centergrid_x) + 700;
		n = floor(centergrid_y) + 450;

		conc[m][n] += concentration;

		//std::cout << centergrid_x << " " << centergrid_y << std::endl;
		//std::cout << m << " " << n << " " << conc[m][n] << std::endl;

		glBegin(GL_QUADS);
		glColor3f(0.9, 0.9, 0.0);
		glVertex2f(centergrid_x - 1, centergrid_y - 1);
		glVertex2f(centergrid_x - 1, centergrid_y + 1);
		glVertex2f(centergrid_x + 1, centergrid_y + 1);
		glVertex2f(centergrid_x + 1, centergrid_y - 1);
		glEnd();

	}


	fclose(inFile);


	// DISPLAY AGENTS' SHAPES

	if (CHASE > 0) {

		int n = CHASE - 1;

		glBegin(GL_POLYGON);

		if (COLORING_TRAFFIC) {

			if (velocity[n] < 0.5)
				glColor3f(1.0, 2 * velocity[n], 0.0);
			else if (velocity[n] >= 0.5 && velocity[n] <= 1)
				glColor3f(-2 * velocity[n] + 2, 1.0, 0.0);
			else
				glColor3f(0.0, 1.0, 0.0);
		}
		if (COLORING_DOSE) {

			if (agentdose[n] < color_orange)
				glColor3f(0.0, 1.0, 0.0);
			else if (agentdose[n] >= color_orange && agentdose[n] <= color_red)
				glColor3f(1.0, 0.5, 0.0);
			else
				glColor3f(1.0, 0.0, 0.0);
		}
		if (COLORING_TRAFFIC + COLORING_DOSE == 0) glColor3f(0.0, 1.0, 0.0);

		glVertex2f(x_position[n] - AGENT_SIZER / 2, y_position[n] + AGENT_SIZER / 2);
		glVertex2f(x_position[n] - AGENT_SIZER / 2, y_position[n] - AGENT_SIZER / 2);
		glVertex2f(x_position[n] + AGENT_SIZER / 2, y_position[n] - AGENT_SIZER / 2);
		glVertex2f(x_position[n] + AGENT_SIZER / 2, y_position[n] + AGENT_SIZER / 2);
		glEnd();

	}
	else {
		for (int n = 0; n < maxID + 1; n++)
		{
			glBegin(GL_POLYGON);

			if (COLORING_TRAFFIC) {

				if (velocity[n] < 0.5)
					glColor3f(1.0, 2 * velocity[n], 0.0);
				else if (velocity[n] >= 0.5 && velocity[n] <= 1)
					glColor3f(-2 * velocity[n] + 2, 1.0, 0.0);
				else
					glColor3f(0.0, 1.0, 0.0);
			}
			if (COLORING_DOSE) {

				if (agentdose[n] < color_orange)
					glColor3f(0.0, 1.0, 0.0);
				else if (agentdose[n] >= color_orange && agentdose[n] <= color_red)
					glColor3f(1.0, 0.5, 0.0);
				else
					glColor3f(1.0, 0.0, 0.0);
			}
			if (COLORING_TRAFFIC + COLORING_DOSE == 0) glColor3f(0.0, 1.0, 0.0);

			glVertex2f(x_position[n] - AGENT_SIZER / 2, y_position[n] + AGENT_SIZER / 2);
			glVertex2f(x_position[n] - AGENT_SIZER / 2, y_position[n] - AGENT_SIZER / 2);
			glVertex2f(x_position[n] + AGENT_SIZER / 2, y_position[n] - AGENT_SIZER / 2);
			glVertex2f(x_position[n] + AGENT_SIZER / 2, y_position[n] + AGENT_SIZER / 2);
			glEnd();
		}
	}

	// GLUT_DOUBLE DISPLAY MODE
	glutSwapBuffers();

}

void saveMousePos(int x, int y)
{
	double t;

	t = double(x) / winw;
	cam_mousex = cam_xmin + t * (cam_xmax - cam_xmin);

	t = double(y) / winh;
	cam_mousey = cam_ymax + t * (cam_ymin - cam_ymax);

	glutPostRedisplay();
}

void myMouse(int button, int state, int x, int y)
{

	saveMousePos(x, y);

	if (button == GLUT_LEFT_BUTTON)
	{

		dragging = (state == GLUT_DOWN);

		glutPostRedisplay();
	}
}

void myMotion(int x, int y)
{
	double old_mousex = cam_mousex;
	double old_mousey = cam_mousey;

	saveMousePos(x, y);

	if (dragging)
	{

		glPushMatrix();
		glTranslatef(cam_mousex - old_mousex, cam_mousey - old_mousey, 0.0f);
		glPopMatrix();

		glFlush();

		//upside += cam_mousey - old_mousey;
		//bottomside += cam_mousey - old_mousey;
		//leftside += cam_mousex - old_mousex;
		//rightside += cam_mousex - old_mousex;

		//gluOrtho2D(leftside, rightside, bottomside, upside);

		glutPostRedisplay();
	}
}

void mouse(GLint X, GLint Y)
{

	ViewX = X;
	ViewY = Y;

	glutPostRedisplay();

}

void MouseWheel(int wheel, int direction, int x, int y)
{
	wheel = 0;
	if (direction == -1 && zoom >= 0.8)
	{
		zoom -= 0.2;

	}
	else if (direction == +1)
	{
		zoom += 0.2;
	}

	glutPostRedisplay();

}

void Keys(int key, int x, int y)
{

	if (key == GLUT_KEY_HOME) {
		G_TIME = INITIAL_TIME;
		HYSPLIT_STEP = 1;
		for (int i = 0; i < AGENT_NUMBER; i++) agentdose[i] = 0;
		glutPostRedisplay();
	}
	//else if (key == GLUT_KEY_RIGHT) {
	//	G_TIME += 500;
	//	glutPostRedisplay();
	//}
	else if (key == GLUT_KEY_PAGE_DOWN) {
		system("pause");
	}
	else if (key == GLUT_KEY_LEFT) {
		leftside += 200 / zoom;
		rightside += 200 / zoom;
		//gluLookAt(0.0, 0.0, 0.0, X, Y, -1.0, 0, 1.0, 0.0);
		gluOrtho2D(leftside, rightside, bottomside, upside);
		glutPostRedisplay();
	}
	else if (key == GLUT_KEY_RIGHT) {
		leftside -= 200 / zoom;
		rightside -= 200 / zoom;
		gluOrtho2D(leftside, rightside, bottomside, upside);
		glutPostRedisplay();
	}
	else if (key == GLUT_KEY_UP) {
		upside += 200 / zoom;
		bottomside += 200 / zoom;
		gluOrtho2D(leftside, rightside, bottomside, upside);
		glutPostRedisplay();
	}
	else if (key == GLUT_KEY_DOWN) {
		upside -= 200 / zoom;
		bottomside -= 200 / zoom;
		gluOrtho2D(leftside, rightside, bottomside, upside);
		glutPostRedisplay();
	}

}

void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	winw = w;
	winh = h;

	//if (w > h)
	//{
	//	cam_xmin = -double(w) / h;
	//	cam_xmax = double(w) / h;
	//	cam_ymin = -1.;
	//	cam_ymax = 1.;
	//}
	//else
	//{
	//	cam_xmin = -1.;
	//	cam_xmax = 1.;
	//	cam_ymin = -double(h) / w;
	//	cam_ymax = double(h) / w;
	//}

	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//gluOrtho2D(cam_xmin, cam_xmax, cam_ymin, cam_ymax);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(leftside, rightside, bottomside, upside);

	glMatrixMode(GL_MODELVIEW);
}


void timer(int)
{
	if (CHECK_G_TIME) {
		char buf[128];
		sprintf(buf, "%f", G_TIME);
		printf("G_TIME: %.7s\n", buf);
	}

	glutPostRedisplay();
	glutTimerFunc(TIME_STEP, timer, 0);

	int i, xx, yy;
	float totaldose = 0;

	for (int n = 0; n < maxID + 1; n++)
	{
		i = 0;
		while (1)
		{

			if (i == pathtime[n].size() - 1) {
				//x_position[n] = 700; 
				//y_position[n] = 450; 
				break;
			}

			if (G_TIME >= pathtime[n][i] && G_TIME < pathtime[n][i + 1])
			{
				x_position[n] = nodeArray[agentpath_node[n][i] - 1][0]
					+ (nodeArray[agentpath_node[n][i + 1] - 1][0] - nodeArray[agentpath_node[n][i] - 1][0])
					* (G_TIME - pathtime[n][i]) / (pathtime[n][i + 1] - pathtime[n][i]);

				//if (CHECK_AGENT_POSITION) std::cout << "[x_pos]: " << x_position[10] << " ";

				y_position[n] = nodeArray[agentpath_node[n][i] - 1][1]
					+ (nodeArray[agentpath_node[n][i + 1] - 1][1] - nodeArray[agentpath_node[n][i] - 1][1])
					* (G_TIME - pathtime[n][i]) / (pathtime[n][i + 1] - pathtime[n][i]);


				xx = floor(x_position[n]) + 700;
				yy = floor(y_position[n]) + 450;

				agentdose[n] += 10000 * conc[xx][yy];
				totaldose += 10000 * conc[xx][yy];

				//std::cout << "[ID: " << n << "] conc: " << agentdose[n] << std::endl;

				//if (CHECK_AGENT_POSITION) std::cout << "[y_pos]: " << y_position[10] << std::endl;

				if (CHASE > 0) {
					if (n == CHASE - 1) {
						printf("Position of vehicle ID(%d): (%07f, %07f)	at %07f (msec)\n"
							, n + 1, x_position[n], y_position[n], G_TIME);
					}
				}

				velocity[n] = (linkArray[agentpath_link[n][i]][2]
					/ (pathtime[n][i + 1] - pathtime[n][i])) / linkArray[agentpath_link[n][i]][3];

				//if(velocity[n] > 10) printf("Velocity of vehicle ID(%d): %f\n", n, velocity[n]);

				if (CHECK_VELOCITY) {
					if (CHASE > 0) {
						if (n == CHASE - 1) {
							printf("Velocity of vehicle ID(%d): %f\n", n + 1, velocity[n]);
						}
					}
					else
						printf("Velocity of vehicle ID(%d): %f\n", n + 1, velocity[n]);
				}
			}
			else if (G_TIME < pathtime[n][0])
			{
				x_position[n] = nodeArray[agentpath_node[n][0] - 1][0];

				//if (CHECK_AGENT_POSITION) std::cout << "[x_pos]: " << x_position[10] << " ";

				y_position[n] = nodeArray[agentpath_node[n][0] - 1][1];

				//if (CHECK_AGENT_POSITION) std::cout << "[y_pos]: " << y_position[10] << std::endl;

				xx = floor(x_position[n]) + 700;
				yy = floor(y_position[n]) + 450;

				agentdose[n] += 10000 * conc[xx][yy];
				totaldose += 10000 * conc[xx][yy];

				if (CHASE > 0) {
					if (n == CHASE - 1) {
						printf("Position of vehicle ID(%d): (%07f, %07f)	at %07f (msec)\n"
							, n + 1, x_position[n], y_position[n], G_TIME);
					}
				}

				velocity[n] = linkArray[agentpath_link[n][0]][3];

				if (CHECK_VELOCITY) {
					if (CHASE > 0) {
						if (n == CHASE - 1) {
							printf("Velocity of vehicle ID(%d): %f\n", n + 1, velocity[n]);
						}
					}
					else
						printf("Velocity of vehicle ID(%d): %f\n", n + 1, velocity[n]);
				}
			}

			i++;
		}
	}

	G_TIME += SPEED_PARAMETER * TIME_STEP;
	totaldosebytime.push_back(totaldose);

	int numorange = 0;
	int numred = 0;

	for (int n = 0; n < maxID; n++) {
		if (agentdose[n] > color_red)
			numred++;
		else if (color_red >= agentdose[n] && agentdose[n] > color_orange)
			numorange++;
	}

	redbytime.push_back(numred);
	orangebytime.push_back(numorange);


	if (HYSPLIT_REALTIME) {
		HYSPLIT_TICK += 1;

		if (HYSPLIT_STEP < SIMULATION_STEP && HYSPLIT_TICK == (int)(60 / SPEED_PARAMETER * TIME_STEP)) {

			HYSPLIT_STEP += 1;
			HYSPLIT_TICK = 0;

		}
	}
	else
		if (HYSPLIT_STEP < SIMULATION_STEP) HYSPLIT_STEP++;



	if (HYSPLIT_STEP == SIMULATION_STEP)
	{

		float sumoftotaldose = 0;

		std::vector<int> shelter;
		std::vector<float> shelterdose;

		int index = 0;


		for (int p = 0; p < maxID + 1; p++)
		{
			if (CHECK_TOTAL_DOSE_OF_AGENT) std::cout << "[ID: " << p + 1 << "] " << agentdose[p];

			if (agentdose[p] > 0.0001) {

				if (CHECK_TOTAL_DOSE_OF_AGENT) std::cout << "   [shelter ID]: " << agentpath_node[p].back();

				for (index = 0; index < shelter.size(); index++) {
					if ((shelter[index] - agentpath_node[p].back()) == 0) {
						shelterdose[index] += agentdose[p];
						break;
					}
				}

				//std::cout << "      " << shelter.size() << std::endl;
				if (index == shelter.size()) {
					shelter.push_back(agentpath_node[p].back());
					shelterdose.push_back(agentdose[p]);
				}

			}

			if (CHECK_TOTAL_DOSE_OF_AGENT) std::cout << std::endl;

			sumoftotaldose += agentdose[p];
		}

		std::cout << std::endl << "[Total dose of agents]: " << sumoftotaldose << std::endl << std::endl;


		if (CHECK_SHELTER_DOSE) {

			for (int n : shelter)
				std::cout << n << " ";

			std::cout << std::endl;
			std::cout << std::endl;

			for (float n : shelterdose)
				std::cout << n << " ";

			std::cout << std::endl;
			std::cout << std::endl;

		}

		if (CHECK_BYTIME) {

			for (float n : totaldosebytime)
				std::cout << n << " ";

			std::cout << std::endl;
			std::cout << std::endl;

			for (int n : orangebytime)
				std::cout << n << " ";

			std::cout << std::endl;
			std::cout << std::endl;

			for (int n : redbytime)
				std::cout << n << " ";

			std::cout << std::endl;
			std::cout << std::endl;

			std::cout << totaldosebytime.size() << std::endl;
			std::cout << orangebytime.size() << std::endl;
			std::cout << redbytime.size() << std::endl;

		}

		char output[256];
		FILE* outFile;

		sprintf(output, "./log/data/properties.txt");
		outFile = fopen(output, "w");

		yr = SIMULATION_START_TIME / 100000000;
		date = SIMULATION_START_TIME / 10000 % 10000;
		hr = SIMULATION_START_TIME / 100 % 100;
		min = SIMULATION_START_TIME % 100;

		fprintf(outFile, "\n");
		fprintf(outFile, " [ MESH v1.0.0 ]\n");
		fprintf(outFile, "\n");
		fprintf(outFile, "Number_of_Agents: %d\n", AGENT_NUMBER);
		fprintf(outFile, "Number_of_Nodes: %d\n", NUM_OF_NODES);
		fprintf(outFile, "Number_of_Links: %d\n", NUM_OF_LINKS);
		fprintf(outFile, "Number_of_Events: %d\n", NUM_OF_EVENT);
		fprintf(outFile, "\n");
		fprintf(outFile, "Start_time_of_Simulation: %04d %02d %02d, %02d:%02d\n", yr, date / 100, date % 100, hr, min);
		fprintf(outFile, "End_time_of_Simulation: %04d %02d %02d, %02d:%02d\n",
			yr, date / 100, date % 100 + SIMULATION_STEP / (60 * 24), hr + SIMULATION_STEP / 60 % 24, min + SIMULATION_STEP % 60);
		fprintf(outFile, "Total_time_of_Simulation: %d days %d hrs %d min 00 sec\n",
			SIMULATION_STEP / (60 * 24), SIMULATION_STEP / 60 % 24, SIMULATION_STEP % 60);
		fprintf(outFile, "\n");
		fprintf(outFile, "Frame_per_seconds: %d\n", FPS);
		fprintf(outFile, "Global_time_step: %f\n", SPEED_PARAMETER * TIME_STEP);
		fprintf(outFile, "\n");
		fprintf(outFile, "Threshold_Orange: %f\n", color_orange);
		fprintf(outFile, "Number_of_Orange: %d\n", orangebytime.back());
		fprintf(outFile, "Threshold_Red: %f\n", color_red);
		fprintf(outFile, "Number_of_Red: %d\n", redbytime.back());
		fprintf(outFile, "\n");
		fprintf(outFile, "Total_dose_of_Agents: %f\n", sumoftotaldose);
		fprintf(outFile, "\n");
		fprintf(outFile, "Percent_of_exposured: %f\n", (float)((orangebytime.back() + redbytime.back()) / (float)AGENT_NUMBER));

		fclose(outFile);

		sprintf(output, "./log/data/shelter.txt");
		outFile = fopen(output, "w");

		fprintf(outFile, "%d\n", shelter.size());
		for (int n : shelter)
			fprintf(outFile, "%d ", n);
		fprintf(outFile, "\n");
		for (int n : shelter)
			fprintf(outFile, "%s ", nodename[n]);
		fprintf(outFile, "\n");
		for (float n : shelterdose)
			fprintf(outFile, "%f ", n);

		fclose(outFile);

		sprintf(output, "./log/data/dosebytime.txt");
		outFile = fopen(output, "w");

		fprintf(outFile, "%d\n", totaldosebytime.size());
		for (float n : totaldosebytime)
			fprintf(outFile, "%f ", n);

		fclose(outFile);

		sprintf(output, "./log/data/colorbytime.txt");
		outFile = fopen(output, "w");

		fprintf(outFile, "%d\n", orangebytime.size());
		for (int n : orangebytime)
			fprintf(outFile, "%d ", n);
		fprintf(outFile, "\n");
		for (int n : redbytime)
			fprintf(outFile, "%d ", n);

		fclose(outFile);

		sprintf(output, "./log/data/dosebyID.txt");
		outFile = fopen(output, "w");

		fprintf(outFile, "%d\n", AGENT_NUMBER);
		for (int n = 0; n < AGENT_NUMBER; n++)
			fprintf(outFile, "%d ", n + 1);
		fprintf(outFile, "\n");
		for (int n = 0; n < AGENT_NUMBER; n++)
			fprintf(outFile, "%f ", agentdose[n]);
		fprintf(outFile, "\n");

		fclose(outFile);

		system("pause");
	}

}


int main(int argc, char** argv)
{

	// READ "NETWORK.XML"

	if (REWRITE_NETWORK) network();
	else open_network();

	// READ "EVENT.XML"

	if (REWRITE_EVENT) event();
	else open_event();

	//for (int n = 0; n < maxID + 1; n++)
	//{
	//	x_position[n] = nodeArray[agentpath_node[n][0] - 1][0];
	//	y_position[n] = nodeArray[agentpath_node[n][0] - 1][1];
	//}

	std::cout << "Press < Enter key > to visualize the simulation" << std::endl << std::endl;

	system("pause");


	if (CHECK_NODENAME) {
		printf("\n\n// Check node name //\n\n");
		for (int i = 0; i < NON; i++) {
			printf("Node ID(%d): %s\n", i + 1, nodename[i]);
		}
	}

	if (CHECK_LINKNAME) {
		printf("\n\n// Check link name //\n\n");
		for (int i = 0; i < NOL; i++) {
			printf("Link ID(%d): %s\n", i + 1, linkname[i]);
		}
	}


	// TRANSFORM TO OPENGL GEOMETRY
	for (int i = 0; i < NUM_OF_NODES; i++)
	{
		nodeArray[i][0] = nodeArray[i][0] * SIZING_PARAMETER + PARALLEL_PARAMETER_X;
		nodeArray[i][1] = nodeArray[i][1] * SIZING_PARAMETER + PARALLEL_PARAMETER_Y;
		if (CHECK_NODE_POSITION) std::cout << std::endl << i << "  Node position <x, y> = <"
			<< nodeArray[i][0] << ", " << nodeArray[i][1] << ">" << std::endl;
	}



	// OPENGL INITIALIZATION

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	glutInitWindowPosition(500, 50);
	glutInitWindowSize(SCREEN_WIDTH, SCREEN_HEIGHT);

	glutCreateWindow("MESH");

	glutDisplayFunc(display); // CALL DISPLAY FUNC
	glutReshapeFunc(reshape); // CALL RESHAPE FUNC
	glutTimerFunc(0, timer, 0); // CALL TIMER FUNC

	glutSpecialFunc(Keys);
	glutMotionFunc(mouse);
	glutMouseWheelFunc(MouseWheel);

	glutMouseFunc(myMouse);
	glutMotionFunc(myMotion);

	init();

	glutMainLoop();


}