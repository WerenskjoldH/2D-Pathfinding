#pragma region Pre-processor Definitions

#define DELTA_TIME 0.03

// Keep these square
#define WINDOW_WIDTH 400
#define WINDOW_HEIGHT 400

// This is used for the number of cells on one axis
// +2 is used to account for the boundary cells
#define ONE_AXIS_CELLS (10 + 2)
#define TOTAL_CELLS ONE_AXIS_CELLS * ONE_AXIS_CELLS
#define CELL_OFFSET WINDOW_WIDTH/(ONE_AXIS_CELLS-2)

#define CELL_BUFFER (CELL_OFFSET/10)

#define DEFAULT_COST 1

#define USE_DFT 0

#define DRAW_VISITED_NODES 0

#pragma endregion

#pragma region Includes

#include <iostream>
#include <vector>

#include "Window.h"

#include "gfxHelper.h"

#pragma endregion

#pragma region Function Declarations

void inputs();
void update();
void draw();
bool getMouseLeftClick();
bool getMouseRightClick();
bool getSKeyPress();
bool getGKeyPress();
bool getSpaceKeyPress();

#pragma endregion

#pragma region Enums

// Enum to define all possible states of a cell
typedef enum {
	EMPTY,
	BOUNDARY,
	PATH,
	DISCOVERED,
	START,
	GOAL
} CELL_TYPE;

#pragma endregion

#pragma region Structs

//// Cell struct ////

typedef struct {
	int arrayX = -1;
	int arrayY = -1;
	int screenX = -1 * CELL_OFFSET/2;
	int screenY = -1 * CELL_OFFSET/2;
	int type = EMPTY;
	int cost = DEFAULT_COST;

	int getArrayPos()
	{
 		return arrayX + arrayY * ONE_AXIS_CELLS;
	}
} cell;

//// Node struct ////

struct node {

	node(float gCost, float hCost, float fCost, cell* c, node* parent) : gCost{ gCost }, hCost{ hCost }, fCost{ fCost }, c{ c }, parent{ parent }
	{}

	float gCost = 0;
	float hCost = 0;
	float fCost = 0;
	bool visited = 0;
	node* parent = NULL;
	cell* c = NULL;
};

//// Grid struct ////

struct grid{
	cell cells[TOTAL_CELLS];

	bool goalExist = 0;
	int goalPosition = 0;

	bool startExist = 0;
	int startPosition = 0;

	~grid()
	{

	}

	void 
	InitCells()
	{
		for (int i = 0; i < ONE_AXIS_CELLS; i++)
			for (int j = 0; j < ONE_AXIS_CELLS; j++)
			{
				int cellPos = i + (j * ONE_AXIS_CELLS);
				if (i == ONE_AXIS_CELLS - 1 || j == ONE_AXIS_CELLS -1 || i == 0 || j == 0)
				{
					cells[cellPos].type = BOUNDARY;
					cells[cellPos].screenY += j * CELL_OFFSET;
					cells[cellPos].screenX += i * CELL_OFFSET;
					cells[cellPos].arrayY = j;
					cells[cellPos].arrayX = i;
				}
				else
				{
					cells[cellPos].screenY += j * CELL_OFFSET;
					cells[cellPos].screenX += i * CELL_OFFSET;
					cells[cellPos].arrayY = j;
					cells[cellPos].arrayX = i;
				}

			}
	}

	cell* getCellFromScreenPosition(int x, int y)
	{
		// We need to add 1 to the cell's discrete value to compensate for the boundary cells off-screen
		int xOffset = int(x / int(CELL_OFFSET)) + 1;
		int yOffset = (int(y / int(CELL_OFFSET)) + 1) * int(ONE_AXIS_CELLS);
		return &cells[xOffset + yOffset];
	}

	cell* getCellDiscrete(int x, int y)
	{
		return &cells[x + y * ONE_AXIS_CELLS];
	}

	void
	drawGrid(SDL_Renderer* renderer)
	{
		// Only draw what is visible, else we are just wasting time
		for (int i = 1; i < ONE_AXIS_CELLS-1; i++)
			for (int j = 1; j < ONE_AXIS_CELLS-1; j++)
			{
				int cellPos = i + (j * ONE_AXIS_CELLS);
				if (cells[cellPos].type == BOUNDARY)
					SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
				else if (cells[cellPos].type == PATH)
					SDL_SetRenderDrawColor(renderer, 100, 100, 255, 255);
				else if( cells[cellPos].type == DISCOVERED)
					SDL_SetRenderDrawColor(renderer, 150, 200, 150, 255);
				else if (cells[cellPos].type == START)
					SDL_SetRenderDrawColor(renderer, 180, 255, 180, 255);
				else if (cells[cellPos].type == GOAL)
					SDL_SetRenderDrawColor(renderer, 200, 100, 100, 255);
				else
					SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);

				gfxDrawSquare(renderer, cells[i + (j * ONE_AXIS_CELLS)].screenX, cells[i + (j * ONE_AXIS_CELLS)].screenY, CELL_OFFSET/2 - CELL_BUFFER);
			}
	}

	void
	resetPath()
	{
		for (int i = 0; i < ONE_AXIS_CELLS; i++)
			for (int j = 0; j < ONE_AXIS_CELLS; j++)
			{
				if (cells[i + j * ONE_AXIS_CELLS].type == PATH || cells[i + j * ONE_AXIS_CELLS].type == DISCOVERED)
					cells[i + j * ONE_AXIS_CELLS].type = EMPTY;
			}

		pathExists = 0;
		pathIndex = 0;
	}

	// A* Pathfinding, disregards diagonals
	void
	pathfindGrid()
	{
		// When the path is found, mark each crossed cell with a type of PATH

		if (!startExist || !goalExist)
			return;

#if USE_DFT

		// Execute A* pathfinding code
		bool v[TOTAL_CELLS];
		int p[TOTAL_CELLS];
		int pI = 0;

		for (int i = 0; i < TOTAL_CELLS; i++)
			v[i] = 0;

		DFT(startPosition, v, p, pI);

		if (pathExists)
		{
			for (int i = 1; i < pI - 1; i++)
			{
				cells[p[i]].type = PATH;
			}
		}

#endif

		Astar();

		// Draw Path
		for (int i = 0; i < pathIndex; i++)
		{
			cells[path[i]].type = PATH;
		}
		

	}

private:

	bool pathExists = 0;
	
	int path[TOTAL_CELLS];
	int pathIndex = 0;

#if !USE_DFT
	int adj[8] = { -1 * ONE_AXIS_CELLS - 1, -1 * ONE_AXIS_CELLS, -1 * ONE_AXIS_CELLS + 1,
				   -1,										   						   1,
					    ONE_AXIS_CELLS - 1,	     ONE_AXIS_CELLS,      ONE_AXIS_CELLS + 1};
#endif

#if USE_DFT

	// Top, right, down, left
	int adj[4] = { -1 * ONE_AXIS_CELLS, 1, ONE_AXIS_CELLS, -1 };
	
	bool 
	DFT(int currentIndex, bool visited[], int path[], int &pathIndex)
	{
		visited[currentIndex] = 1;
		path[pathIndex] = currentIndex;
		pathIndex++;

		if (currentIndex == goalPosition)
		{
			// Path found
			printf("Path found\n");
			pathExists = 1;
			return 1;
		}
		else
		{
			// Check all adjacent verticies to current vertex
			for (int i = 0; i < 4; i++)
			{
				int offset = currentIndex + adj[i];
				cell* c = &cells[offset];
				if(visited[offset] == 0)
					if (c->type == BOUNDARY)
					{
						visited[offset] = 1;
						continue;
					}
					else
					{
						if (DFT(offset, visited, path, pathIndex) == 1)
							return 1;
					}

			}
		}

		pathIndex--;
		visited[currentIndex] = false;
	}

#endif

	float getDistance(int x, int x1, int y, int y1)
	{
		float xDistSquared = std::powf(float(x1 - x), 2);
		float yDistSquared = std::powf(float(y1 - y), 2);
		return std::sqrtf(xDistSquared + yDistSquared);
	}

	node* createNode(int x, int y, node* parent)
	{
		float gCost = getDistance(x, parent->c->arrayX, y, parent->c->arrayY) + parent->gCost;
		float hCost = getDistance(x, cells[goalPosition].arrayX, y, cells[goalPosition].arrayY);
		float fCost = gCost + hCost;
		return new node(gCost, hCost, fCost, &cells[x + y * ONE_AXIS_CELLS], parent);
	}

	bool checkVisited(std::vector<node*> &d, int x, int y)
	{
		for (auto it = d.begin(); it != d.end(); it++)
			if ((**it).c->arrayX == x && (**it).c->arrayY == y)
				return 1;
		return 0;
	}

	node* fetchNode(std::vector<node*>& d, int x, int y)
	{
		for (auto it = d.begin(); it != d.end(); it++)
			if ((**it).c->arrayX == x && (**it).c->arrayY == y)
				return (*it);
		return NULL;
	}

	node* findLowestFCost(std::vector<node*>& d)
	{
		if (d.empty())
			return NULL;

		node* lowest = NULL;

		for (auto it = d.begin(); it != d.end(); it++)
		{
			if ((**it).visited)
				continue;

			if ((**it).c->type == GOAL)
				return (*it);

			if (lowest == NULL)
			{
				lowest = (*it);
				continue;
			}

			if((**it).fCost < lowest->fCost)
				lowest = (*it);
		}
		
		return lowest;
	}

	void addAndUpdateAdjacents(std::vector<node*> &d, node* n)
	{
		for (int i = 0; i < 8; i++)
		{
			if (n == NULL)
				break;

			int offset = n->c->getArrayPos() + adj[i];
			if (cells[offset].type != BOUNDARY && !checkVisited(d, cells[offset].arrayX, cells[offset].arrayY))
			{
				node* fetchedNode = fetchNode(d, cells[offset].arrayX, cells[offset].arrayY);
				if (fetchedNode == nullptr) // We add a new node
				{
					d.push_back(createNode(cells[offset].arrayX, cells[offset].arrayY, n));
				}
				else // We update the existing node
				{
					if (fetchedNode->fCost < n->fCost)
						continue;

					// update fetchedNode
					fetchedNode->gCost = getDistance(fetchedNode->c->arrayX, n->c->arrayX, fetchedNode->c->arrayY, n->c->arrayY) + n->gCost;
					fetchedNode->hCost = getDistance(fetchedNode->c->arrayX, cells[goalPosition].arrayX, fetchedNode->c->arrayY, cells[goalPosition].arrayY);
					fetchedNode->fCost = fetchedNode->gCost + fetchedNode->hCost;
					fetchedNode->parent = n;
				}

			}
		}
	}

	template <typename T>
	inline void freeVector(std::vector<T> &v) 
	{
		// Free all pointers
		for (std::vector<T>::iterator i = v.begin(); i != v.end(); ++i)
			delete* i;

		// Not necessary, but for completeness
		v.clear();
	}

	void
	Astar()
	{
		std::vector<node*> discovery;

		node* startingNode = new node(0, getDistance(cells[startPosition].arrayX, cells[goalPosition].arrayX, cells[startPosition].arrayY, cells[goalPosition].arrayY), getDistance(cells[startPosition].arrayX, cells[goalPosition].arrayX, cells[startPosition].arrayY, cells[goalPosition].arrayY), &cells[startPosition], nullptr);
		discovery.push_back(startingNode);

		do
		{
			node* n = findLowestFCost(discovery);
			if (n == NULL) // There are no more options
				break;

			if (n->c->type == GOAL) // We found the goal
			{
				printf("Goal has been found!\n");
				pathExists = 1;

				node* pathNode = n->parent;
				while (pathNode->c->type != START)
				{
					// Store the path
					path[pathIndex++] = pathNode->c->getArrayPos();
					pathNode = pathNode->parent;
				}

				freeVector(discovery);

				return;
			}
			else
			{
				addAndUpdateAdjacents(discovery, n);
				n->visited = true;
#if DRAW_VISITED_NODES
				if(n->c->type != START)
					n->c->type = DISCOVERED;
#endif
			}


		} while (!discovery.empty());

		printf("No path has been found.");
		
		// Free all pointers
		for (std::vector<node*>::iterator i = discovery.begin(); i != discovery.end(); ++i)
			delete* i;

		// Not necessary, but for completeness
		freeVector(discovery);

	}
	
};

#pragma endregion

#pragma region Global Variables

window* gameWindow;

grid* Grid;

float iTime = 0;

/// Controls
// S - Place start at location of mouse
// G - Place goal at location of mouse
// Space - Create path
// Left Click - Place wall/Remove wall
// Right Click - Get node's values

// Mouse
bool leftClick = 0;
bool prevLeftClick = 0;

bool rightClick = 0;
bool prevRightClick = 0;

int mouseX = 0;
int mouseY = 0;

// Keys
bool sKeyPress = 0;
bool sKeyPrev = 0;

bool gKeyPress = 0;
bool gKeyPrev = 0;

bool spaceKeyPress = 0;
bool spaceKeyPrev = 0;

#pragma endregion

#pragma region Function Definitions

int
main(int args, char* argv[])
{
	gameWindow = new window("2D Pathfinding - Hunter Werenskjold", WINDOW_WIDTH, WINDOW_HEIGHT);

	Grid = new grid();
	Grid->InitCells();

	SDL_Event e;
	while (gameWindow->checkIfRunning())
	{	
		// Set previous mouse and button states
		prevLeftClick = leftClick;
		prevRightClick = rightClick;
		sKeyPrev = sKeyPress;
		gKeyPrev = gKeyPress;
		spaceKeyPrev = spaceKeyPress;

		while (SDL_PollEvent(&e))
		{
			if ((e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) || e.type == SDL_QUIT)
			{
				gameWindow->stopWindow();
				break;
			}

			if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
			{
				leftClick = 1;
				continue;
			}
			else if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT)
			{
				leftClick = 0;
				continue;
			}

			if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_RIGHT)
			{
				rightClick = 1;
				continue;
			}
			else if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_RIGHT)
			{
				rightClick = 0;
				continue;
			}

			if (e.type == SDL_MOUSEMOTION)
			{
				SDL_GetMouseState(&mouseX, &mouseY);
				continue;
			}

			if (e.type == SDL_KEYDOWN)
			{
				if (e.key.keysym.sym == SDLK_s)
					sKeyPress = 1;
				else if (e.key.keysym.sym == SDLK_g)
					gKeyPress = 1;
				else if (e.key.keysym.sym == SDLK_SPACE)
					spaceKeyPress = 1;

				continue;
			}
			
			if (e.type == SDL_KEYUP)
			{
				if (e.key.keysym.sym == SDLK_s)
					sKeyPress = 0;
				else if (e.key.keysym.sym == SDLK_g)
					gKeyPress = 0;
				else if (e.key.keysym.sym == SDLK_SPACE)
					spaceKeyPress = 0;

				continue;
			}

		}

		if (!gameWindow->checkIfRunning())
			break;

		inputs();

		update();

		draw();

		iTime += DELTA_TIME;
	}

	delete gameWindow;
	delete Grid;

	return 0;
}

void
inputs()
{
	// Just incase
	if (mouseX < 0 || mouseX > WINDOW_WIDTH || mouseY < 0 || mouseY > WINDOW_HEIGHT)
		return;

	if (getMouseLeftClick())
	{
		printf("Click detected @ <%i, %i>\n", mouseX, mouseY);

		// This is kinda unnecessary to store it as a variable, but it looks so much nicer.
		cell* c = Grid->getCellFromScreenPosition(mouseX, mouseY);

		Grid->resetPath();

		if (c->type == 0)
			c->type = BOUNDARY;
		else if (c->type == 1)
			c->type = EMPTY;
	}

	if (getSKeyPress())
	{
		cell* c = Grid->getCellFromScreenPosition(mouseX, mouseY);

		Grid->resetPath();

		if (c->type == GOAL)
			return;

		if (c->type == START)
		{
			c->type = EMPTY;
			Grid->startPosition = -1;
			Grid->startExist = 0;
			return;
		}

		if (Grid->startExist)
			Grid->cells[Grid->startPosition].type = EMPTY;
		
		c->type = START;
		Grid->startPosition = c->arrayX + c->arrayY * ONE_AXIS_CELLS;
		Grid->startExist = 1;
	}

	if (getGKeyPress())
	{
		cell* c = Grid->getCellFromScreenPosition(mouseX, mouseY);

		Grid->resetPath();

		if (c->type == START)
			return;

		if (c->type == GOAL)
		{
			c->type = EMPTY;
			Grid->goalPosition = -1;
			Grid->goalExist = 0;
			return;
		}
		
		if (Grid->goalExist)
			Grid->cells[Grid->goalPosition].type = EMPTY;		

		c->type = GOAL;
		Grid->goalPosition = c->getArrayPos();
		Grid->goalExist = 1;
	}

	if (getSpaceKeyPress())
	{
		Grid->resetPath();
		Grid->pathfindGrid();
	}
}

void 
update()
{
	
}

void 
draw()
{
	SDL_Renderer* renderer = gameWindow->getRenderer();

	Grid->drawGrid(renderer);

	// Used to debug speed of program - Very brutish way of doing this, but like it works for what I need
	//SDL_SetRenderDrawColor(renderer, 200, 100, 100, 255);
	//gfxDrawBrenCircle(renderer, 200 + 10.f * sinf(iTime), 200 + 10.f * cosf(iTime), 10, true);

	gameWindow->renderWindow();
}

bool 
getMouseLeftClick()
{
	if (leftClick == 1 && prevLeftClick == 0)
		return true;
	return false;
}

bool 
getMouseRightClick()
{
	if (rightClick == 1 && prevRightClick == 0)
		return true;
	return false;
}

bool 
getSKeyPress()
{
	if (sKeyPress == 1 && sKeyPrev == 0)
		return true;
	return false;
}

bool 
getGKeyPress()
{
	if (gKeyPress == 1 && gKeyPrev == 0)
		return true;
	return false;
}

bool
getSpaceKeyPress()
{
	if (spaceKeyPress == 1 && spaceKeyPrev == 0)
		return true;
	return false;
}

#pragma endregion