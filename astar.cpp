/*-
 * Copyright (c) 2011 Forrest Porter. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of 
 * conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY FORREST PORTER ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FORREST PORTER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those of the
 * authors and should not be interpreted as representing official policies, either expressed
 * or implied, of Forrest Porter.
-*/

/*- INFO -
 * This program demonstrates A* pathfinding algorithm. I programmed it quickly to learn about
 * this algorithm, instead of focusing on design. I have left many comments in order to make
 * reading the code easier.
 *
 * ./astar
 * This will show what aStar() does as it runs (delayed).
 *
 * ./astar -r
 * This won't spend any time on it, to show results quickly.
 *
 * Some suggestions on making the code better:
 *
 * 1) Get rid of the global variables.
 * 2) Spread out the code on multiple files (greater read ability).
 * 3) Perhaps integrate aStar() with STL more (reusability).
-*/

#include <iostream>
#include <stdlib.h>
#include <ctime>
#include <math.h>
#include <list>

#ifdef _WIN32
#include <conio.h>
#else
#include <unistd.h>
#include <termios.h>

/*-- Start utility function declations --*/

// Not all OS default to taking in a single character (end on return key), so make our own 
// function even (<curses.h> doesn't solve this issue). Replaces <conio.h> getch();
int getch();
#endif

// clear the console screen
void clearScreen();

void wait(float seconds);

/*-- End utility function declations --*/

/*-- Start new structs  --*/

// nodeObj helps create the start/player and target destination points
struct nodeObj {
    union  {
	struct  {
	    int X;
	    int Y;
	};
	int pos[2];
    };
    const char c;
    nodeObj(char C) : c(C), X(1), Y(1) {};
};

/*-- End new structs  --*/

/*-- Start basic function declations --*/

// default initilization of the map, or map of nodes
void setMap();

// clear the map of everything except blocks and the start/player
void clearMap();

// draw the map to console, clears the screen before hand
void printMap();

// 1/10 chance to return true, for creating random blocks
bool randBlock();

// read in a character, if it's a char that gives a position on the map then print it out
// and return the X/Y value associated with it. If 'Q' is read, returns -1. This function
// does not end unless a correct value is inserted.
int readChar();

// this function prompts user to initilized a nodeObj
void setObject(const char* name, nodeObj &obj);

// adds and new line and quits
void quit();

/*-- End basic function declations --*/

/*-- Start A* pathfinding functions declarations --*/

// dist() is the H function
// H is distance to target
// uses: Euclidean distance; sqrt((x-X)^2+(y-Y)^2)
float dist(int X, int Y, const nodeObj &obj);

// the function to perform A* pathfinding algorithm
void aStar(const nodeObj &start, const nodeObj &end);

/*-- End A* pathfinding functions declarations  --*/

/*-- Start global variables  --*/

// default map size is 22, this program will only work with this size
// first row and column == ' ' + mapNumbers[], last is '\0', for printf("%s")
// so it's actually 20x20.
const int mapSize=22;

// play speed
float waitSpeed = 0.75f;

// realtime
bool realtime = false;

// map X/Y key is 0-J (20 chars) + end string '\0'
const char mapNumbers[mapSize-1]={'0','1','2','3','4','5','6','7','8','9',
				  'A','B','C','D','E','F','G','H','I','J','\0'};

const char  
	    // shows the node not yet checked
	    open='+', 
	    
	    // shows the nodes checked
	    closed='-', 

	    // shows objects players can't go through
	    block='#', 

	    // shows an empty space
	    empty=' ',

	    // shows the path taken
	    path='*';

// map [Y][X], nodes are boxes, or a grid map.
char map[mapSize][mapSize]={0};

nodeObj 
	// our player
	player('@'),

	// our target destination
	target('$');

/*-- End global variables  --*/

/*-- Start new classes --*/

// aNode, for our two lists in A*()
class aNode {
private:
    // never call delete on parent
    // this is used to point to parent
    // not create a new aNode
    aNode* _parent;
    
    int 
    // true dist from start/player
    _G;
    
    float
    // dist from target (est.)
    _H;
    
public:
    
    /*const*/ aNode* Parent() {
	//return static_cast<const aNode*>(_parent);
	return _parent;
    }
    
    // true distance from start/player
    int G() const {
	return _G;
    }
    
    // distand to target (est.)
    float H() const {
	return _H;
    }
    
    // F = G+H
    float F() const {
	return G()+H();
    }
    
    // for sorting purposes
    static bool compareNodes(const aNode* node1,const aNode* node2) {
	return (node1->F() < node2->F());
    }
    
    // position on map
    const int X,Y;
    
    aNode(int x, int y) 
    : _parent(NULL), _G(0), _H(dist(x,y,target)), X(x), Y(y) 
    {}
    aNode(int x, int y, aNode* Parent) 
    : _parent(Parent), _G(Parent->G()+1), _H(dist(x,y,target)), X(x), Y(y) 
    {
	//if(_parent->X != X && _parent->Y != Y) --_G;
    }
};

/*-- End new classes --*/

// our nodes
//aNode **mapNodes =;

/*=== MAIN ===*/

int main(int argc, char* argv[]) {
    
    // ./astar -r
    // disables wait and extra symbols
    if(argc==2)
	if(argv[1][0]=='-')
	    if(argv[1][1]=='r')
		realtime = true;
    
    // seed random function
    srand(time(NULL));
    
    // initilize the map
    setMap();
    
    // asks for player, refreshes map if incorrect
    setObject("player",player);
    
    while(true) {
	
	// asks for target destination, refreshes map if incorrect
	setObject("target",target);
    
	// print updated map
	printMap();
    
	// start the a* pathfinding
	aStar(player, target);
	
	// hold here so user may look at screen
	printf("Press a key to continue...");
	getch();
	
	// clear extra symbols from map
	clearMap();
    }
}

void clearScreen() {
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
}

void setMap() {
    
    // dont' want to set first square twice, so initilize it here
    map[0][0]=' ';
    
    // assign mapNumbers or X,Y char values
    for (int i=1; i<mapSize; ++i) {
	map[i][0]=mapNumbers[i-1];
	map[0][i]=mapNumbers[i-1];
    }
    
    // randomly add blocks
    for (int i=1; i<mapSize; ++i) {
	for (int j=1; j<mapSize-1; ++j) {
	    map[i][j]=randBlock() ? block : empty;
	}
    }
}

void clearMap() {
    for (int i=1; i<mapSize; ++i) {
	for (int j=1; j<mapSize-1; ++j) {
	    // remove non-blocks and non-player characters
	    if(map[i][j]!=block && map[i][j]!=player.c) map[i][j]=empty;
	}
    }
}

void printMap() {
    // make draw area clear
    clearScreen();
    
    // add a newline
    putchar('\n');
    
    // draw the map, [(tab)(string)(newline)]
    for (int i=0; i<mapSize; ++i) {
	printf("\t%s\n",map[i]);
    }
}

bool randBlock() {
    // 10% chance for block
    if(rand()%10==0)return true;
    return false;
}

int readChar() {
    
    // input
    char input;

    do{
	// read char
	input = getch();
	
	// if it's a number
	for (int i=0; i<10; ++i) {
	    if(input==mapNumbers[i]){putchar(input);return i+1;}
	}
	
	// else it's a letter, make sure it's capitalized
	input=toupper(input);
	
	// look through the letters
	for (int i=10; i<mapSize-2; ++i) {
	    if(input==mapNumbers[i]){putchar(input);return i+1;}
	}
	
	// keep going until input is valid or 'Q'
    } while (input!='Q');
    
    // return -1 on 'Q'
    return -1;
}

void setObject(const char* name, nodeObj &obj) {
    do {
	// reprint the map
	printMap();
	
	// show name of object, prompt for X, and quit option
	printf("Set %s position, Q to exit\n\tX:",name);
	
	// read X until it's valid, or 'Q'
	if((obj.X = readChar())==-1) quit();
	
	// prompt for Y
	printf("\tY:");
	
	// read Y until it's valid, or 'Q'
	if((obj.Y = readChar())==-1) quit();
	
	// newline
	putchar('\n');
	
	// until it's on a valid empty square
    } while (map[obj.Y][obj.X]!=empty);
    
    // assign it to the map
    map[obj.Y][obj.X]=obj.c;
}

#ifndef _WIN32
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;
    
    // get old settings
    tcgetattr(STDIN_FILENO, &oldt);
    
    // copy old settings
    newt = oldt; 
    
    // change new settings
    newt.c_lflag &= ~(ICANON | ECHO);
    
    // apply new settings, NOW
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    // get single char
    ch = getchar();
    
    // apply old settings, NOW
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    // return char
    return ch;
}
#endif

void wait ( float seconds )
{
    clock_t endwait;
    endwait = clock () + (int)(seconds * CLOCKS_PER_SEC) ;
    while (clock() < endwait) {}
}

void quit() {
    
    // newline
    putchar('\n');
    
    // exit program
    exit(0);
}

float dist(int X, int Y, const nodeObj &obj) {
    // uses: Euclidean distance; sqrt((x-X)^2+(y-Y)^2)
    return sqrt(pow((float)(X - obj.X),2) + pow((float)(Y - obj.Y),2));
}

// make sure it's a valid node position
bool nodeCheck(int X, int Y) {
    if(X > 0 && Y > 0 && X < mapSize-1 && Y < mapSize-1 &&
	map[Y][X]!=block && map[Y][X]!=player.c)return true;
    return false;
}

void replaceNode(int X, int Y, aNode* parent, std::list<aNode*> &nodeList) {
    using std::list;
    
    // look through all the items
    list<aNode*>::iterator it;
    for ( it=nodeList.begin() ; it != nodeList.end(); it++ )
	
	// if it's the right one
	if(((aNode*)*it)->X == X && ((aNode*)*it)->Y == Y) {
	    
	    // check if it's worth it
	    if(parent->G() < (((aNode*)*it)->G()-1)) {
		
		// update it
		delete *it;
		*it = new aNode(X,Y,parent);
	    }
	    
	    // break
	    break;
	}
}

// if it runs accross a previous node and it has a lower F() then update it
// otherwise add it to the Open list for future use
void updateLists(int X, int Y, std::list<aNode*> &Open,std::list<aNode*> &Closed) {
    switch(map[Y][X]) {
	case open:
	    replaceNode(X,Y,Open.front(),Open);
	    break;
	case closed:
	    replaceNode(X,Y,Open.front(),Closed);
	    break;
	default:
	    Open.push_back(new aNode(X,Y,Open.front())); 
	    map[Y][X]=open; 
	    // refresh map, and delay
	    if(!realtime){
		printMap();
		wait(waitSpeed);
	    }
	    break;
    }
}

// adds valid "friendly" nighbors
void addNeighbors(std::list<aNode*> &Open,std::list<aNode*> &Closed) {
    // first do vertical checks
    for(int i=-1;i<2;i+=2) {
	
	// top left, bot right
	if(nodeCheck(Open.front()->X+i,Open.front()->Y-i)) {
	    updateLists(Open.front()->X+i,Open.front()->Y-i,Open,Closed);
	}
	// bot left, top right
	if(nodeCheck(Open.front()->X+i,Open.front()->Y+i)) {
	    updateLists(Open.front()->X+i,Open.front()->Y+i,Open,Closed);
	}
    }
    // then horizontal
    for(int i=-1;i<2;i+=2) {
	// left, right
	if(nodeCheck(Open.front()->X+i,Open.front()->Y)) {
	    updateLists(Open.front()->X+i,Open.front()->Y,Open,Closed);
	}
	// bot, top
	if(nodeCheck(Open.front()->X,Open.front()->Y+i)) {
	    updateLists(Open.front()->X,Open.front()->Y+i,Open,Closed);
	}
    }
}

void aStar(const nodeObj &start, const nodeObj &end) {
    using std::list;
    
    // if found the target
    bool targetFound = false;
    
    // create an open and closed list of (A*) nodes
    list<aNode*>Open;
    list<aNode*>Closed;
    
    // add the start node
    Open.push_back(new aNode(start.X,start.Y));
    
    do {
	
	// sort by F()
	Open.sort (aNode::compareNodes);
	
	// if done
	if(Open.front()->X == end.X && Open.front()->Y == end.Y) {
	    
	    // change player position, set player on map
	    map[player.Y=end.Y][player.X=end.X]=player.c;
	    
	    // refresh map, and delay
	    if(!realtime){
		printMap();
		wait(waitSpeed);
	    }
	    
	    // assigne nPtr to last child
	    aNode* nPtr = Open.front();
	    
	    // loop to player/start and change the characters along the way
	    while(nPtr->Parent()) {
		nPtr=nPtr->Parent();
		map[nPtr->Y][nPtr->X]=path;    
		// refresh map, and delay
		if(!realtime){
		    printMap();
		    wait(waitSpeed);
		}
	    }
	    targetFound =true;
	    break;
	} else /*no done*/{
	    // add neighbors
	    addNeighbors(Open,Closed);
	    // add it to closed
	    Closed.push_back(Open.front());
	    // add closed symbol to map
	    map[Open.front()->Y][Open.front()->X]=closed;
	    // refresh map, and delay
	    if(!realtime){
		printMap();
		wait(waitSpeed);
	    }
	    //delete Open.front();
	    // remove it from open
	    Open.remove(Open.front());
	}
    // while still searching
    } while(Open.size() > 0);
    
    // refresh map, and delay
    printMap();
    if(realtime)wait(waitSpeed);
    
    if(!targetFound) {
	map[player.Y][player.X]=player.c;
	printf("No path to target was found\n");
    }
    
    // delete all the objects, no leakage
    list<aNode*>::iterator it;
    for ( it=Open.begin() ; it != Open.end(); it++ )
	delete *it;
    for ( it=Closed.begin() ; it != Closed.end(); it++ )
	delete *it;
}

