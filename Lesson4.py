# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, x, y]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 1],
        [0, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    open=[]
    x=init[0]
    y=init[1]
    g_value=0
    grid[x][y]=-1
    open.append([g_value,x,y])
    
    while(open[0][1]!=goal[0] or open[0][2]!=goal[1]):
        g_value=open[0][0]
        remove=0
        for i in range(len(open)):
            if (g_value>=open[i][0]):
                g_value=open[i][0]
                x=open[i][1]
                y=open[i][2]    
                remove=i
        g_value+=cost
        count=0
        for move in delta:
            x_post=x+move[0]
            y_post=y+move[1]
            if(x_post>=0 and y_post>=0 and x_post<=len(grid)-1 and y_post<=len(grid[0])-1 and grid[x_post][y_post]==0):       
                grid[x_post][y_post]=-1
                open.append([g_value,x_post,y_post])
                count=1
        if count==0:
            open.pop(remove)
            if len(open)==0:
                return 'fail'
    path=[]
    for i in open[0]:            
        path.append(i)
        

    return path
print search(grid,init,goal,cost)
