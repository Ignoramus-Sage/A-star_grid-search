class pointInfo():
    """Class to store grid points as graph nodes.
    """
    def __init__(self, x, y, val, px=-1, py=-1):
        """Function to intialise points info.

        :param int x:
            x-coordinate of the point
        :param int y:
            y-coordinate of the point
        :param int val:
            The value of the point in the grid (0 or 1)
        :param int px:
            x-coordinate of the parent point
        :param int py:
            y-coordinate of the parent point
        :returns:
            None.
        :rtype:
            None
        """
        self.x = x
        self.y = y
        self.val = val
        self.px = px
        self.py = py
        self.fval = float('inf')
        self.heur = float('inf')
        self.cost = float('inf')

    def __eq__(self, other) -> bool:
        """Function to assess if two points are equal.
        :param infoPoint self:
            The first point
        :param infoPoint other:
            The second point
        :returns:
            Returns True if the two points are the same, False otherwise.
        :rtype:
            bool
        """
        return self.x == other.x and self.y == other.y

    def isUnblockedCell(self) -> bool:
        """Function to assess a point is blocked.
        :param infoPoint self:
            The point to check
        :returns:
            Returns True if the point is unblocked, False otherwise.
        :rtype:
            bool
        """
        return self.val != 1


def isValidCell(point_coor, grid_size) -> bool:
    """Assess if a given point is within the grid.
    :param array-like point_coor:
        Array-like object of size 2 that holds the x and y
        coordinates of a given point.
    :param array-like grid_size:
        Array-like object of size 2 taht holds the number
        of rows and cols in the grid.
    :returns:
        True if the point is within the grid, False otherwise.
    :rtype:
        bool
    """
    x, y = point_coor  # get the coordinates of the point
    rows, cols = grid_size  # get the rows and cols
    return (x >= 0) and (x < rows) and (y >= 0) and (y < cols)


def heuristic(a, b, method='Manhattan') -> float:
    """Function to compute distance to the goal.

    Used to determine which point in the grid is nearer
    to the goal.
    :param pointInfo a:
        The first point
    :param pointinfo b:
        The second point.
    : param string method:
        the metric used too compute the distance.
    :returns:
        The distance between the points.
    :rtype:
        float
    """
    assert method in ["Manhattan", "Euclidean", "Diagonal"],        "Method can only be one of the following : Manhattan,        Euclidean or Diagonal"
    if method == 'Manhattan':
        distance = abs(a.x - b.x) + abs(a.y - b.y)
    elif method == 'Euclidean':
        distance = ((a.x-b.x)**2 + (a.y-b.y)**2)**0.5
    else:
        distance = abs(a.x-b.x) + abs(a.y-b.y) +             (2 ** 0.5 - 2) * min(abs(a.x-b.x), abs(a.y-b.y))
    return distance


def backtrackPath(pointsGrid, stop, verbose=True) -> list:
    """backtrack through points to find path.

   Used to loop through the grid and return the path.

    :param list of lists of pointInfo points_grid:
        A grid that holds all relevant info on the points.
    :param infoPoint stop:
        The end point to reach
    :param bool verbose:
        Whether or not too print the path
    :returns:
        the path from the start to the goal.
    :rtype:
        list
    """
    # Intialise the path to empty
    # get the end point to backtrack
    row, col, path = stop.x, stop.y, []
    # backtrack through the grid until reaching the start
    # the start is characterised by being itself's parent
    while not (pointsGrid[row][col].px == row
               and pointsGrid[row][col].py == col):
        path.append([row, col])
        tempRow = pointsGrid[row][col].px
        tempCol = pointsGrid[row][col].py
        row = tempRow
        col = tempCol
    path.append([row, col])
    # reverse the path
    path.reverse()
    # print if verbose
    if verbose:
        literal = ''
        for cell in path:
            literal + str(cell[0])+str(cell[1])+'->'
        print('Path is ', literal)
    return path


def myPathPlanning(grid, start, stop):
    """Executes A* algorithm on a grid.

    Determines if a path can be found between the start and
    the stop given by the user.

    :param list of list grid:
        A list of list that consists of 0 and 1.
    :param list start:
        A list including the x and y coordinates of the
        start point.
    :param list stop:
        A list including the x and y coordinates of the
        stop point.
    :returns:
        The path if found otherwise no output.
    :rtype:
        list
    """
    # get the number rows and cols of the grid
    gridSize = (len(grid), len(grid[0]))
    # Convert the grid cells into points class
    pointsGrid = [[pointInfo(i, j, grid[i][j]) for j in range(gridSize[0])]
                  for i in range(gridSize[1])]
    # We check if the Start, stop are not in the grid
    if not isValidCell((start[0], start[1]), gridSize):
        print("The start is out of the grid")
        return None
    if not isValidCell((stop[0], stop[1]), gridSize):
        print("The stop is out of the grid")
        return None
    # set the start node on the grid
    # set the stopping criteria as mentioned in the
    # backtrack function (itself's parent)
    # set statistics to 0
    x, y = start[0], start[1]
    pointsGrid[x][y].px, pointsGrid[x][y].py = x, y
    pointsGrid[x][y].val = grid[x][y]
    pointsGrid[x][y].fval = 0
    pointsGrid[x][y].cost = 0
    pointsGrid[x][y].heur = 0
    # convert the stop to pointInfo class for ease
    stop = pointInfo(stop[0], stop[1], grid[stop[0]][stop[1]])
    # check if the start ot the stop are blocked
    if not pointsGrid[x][y].isUnblockedCell():
        print("The start is a blocked cell")
        return None
    if not stop.isUnblockedCell():
        print("The stop is a blocked cell")
        return None
    # Early stopping if the start is the stop
    if pointsGrid[x][y] == stop:
        print("stop already reached, nothing to do\n")
        return None
    # Intialise an empty closed list to hold the cells for later
    # initialise a boolean variable for to see if we reached the stop
    openList, reached = [pointsGrid[x][y]], False
    # Intilaise a boolean closed list to keep track of visited cells
    closedList = [[False for i in range(gridSize[0])]
                  for j in range(gridSize[1])]
    # Loop while there more cells to explore
    while len(openList) > 0:
        # discard the promising cell and retrieve it
        cell = openList.pop(0)
        x, y = cell.x, cell.y
        closedList[x][y] == True
        # generate all neighbours coordinates
        neighbourPosList = [[x+i, y+j] for i in range(-1, 2) for j in range(-1,2)]
        neighbourPosList.remove([x, y])
        # Loop through neighbours
        for neighbourPos in neighbourPosList:
            xn, yn = neighbourPos # get x and y 
            # Only check neighbour if it is valid
            if isValidCell(neighbourPos, gridSize):
                # Check if the current cell is the Stop
                if pointsGrid[xn][yn] == stop:
                    # Set the parents of the Stop cells
                    pointsGrid[xn][yn].px = cell.x
                    pointsGrid[xn][yn].py = cell.y
                    print("Stop reached")
                    # backtrack to get path
                    path = backtrackPath(pointsGrid, stop)
                    reached = True
                    return reached, path
                # Ignore cell if it is blocked or in the closed list
                # Else compute statitic and add to the open list
                # if it is in the open list, update statistics
                else :
                    if not closedList[xn][yn] and pointsGrid[xn][yn].isUnblockedCell(): 
                        # compute cost and heuristic values
                        cost = pointsGrid[cell.x][cell.y].cost + heuristic(cell, pointsGrid[xn][yn], method='Euclidean')
                        heur = heuristic(pointsGrid[xn][yn], stop)
                        # update statistics
                        if pointsGrid[xn][yn].fval == float('inf') or pointsGrid[xn][yn].fval > heur+cost:
                            # Update the details of the cell
                            pointsGrid[xn][yn].fval = heur+cost
                            pointsGrid[xn][yn].cost = cost
                            pointsGrid[xn][yn].heur = heur
                            pointsGrid[xn][yn].px = cell.x
                            pointsGrid[xn][yn].py = cell.y
                            # Add to openList
                            openList.append(pointsGrid[xn][yn])
    # if no path can be found
    if not reached:
        print("Failed to find a path")

    return None


if __name__ == "__main__":

    grid = [ [ 1,0,1,0,0 ], [ 0,0,0,0,1 ], [ 1,0,1,1,0 ], [ 0,0,0,0,0 ], [ 0,1,0,1,0 ] ]
    start = [4, 0]
    stop = [0, 4]
    _, path = myPathPlanning(grid, start, stop)
    print(path)

