########## Group-Members Name #############
##                                       ##                             
##  1. Abraha,Eliyas Kidanemmariam       ##
##  2. Leaku, Goitom Abraha              ##
##                                       ##
## ########################################

# import necessary package
import PIL.Image as image
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from math import sqrt


print("Please Select Connectivity Type ")
print("\n Enter 1 to use 4-point Connectivity");
print(" Enter 2 to use 8-point Connectivity");
print(" Enter 3 to use Euclidean distance ");

start = (10,10)
goal = (90,70)
Q = 3
motions = []
conn_type = 1 # default connectivivity type
conn_type = input("")
# Select Connectivity Type
match conn_type:
    case '1': # 4-point connectivity
        motions = [(-1,0),(1,0),(0,-1),(0,1)]
    case '2': # 8-point connectivity
        motions = [(-1,0),(1,0),(0,-1),(0,1),(1,1),(-1,-1),(1,-1),(-1,1)]
    case '3': # Euclidean Distance
        motions = [(-1,0),(1,0),(0,-1),(0,1),(1,1),(-1,-1),(1,-1),(-1,1)]
    case _: # default 4-point connectivity 
        motions = [(-1,0),(1,0),(0,-1),(0,1)]
        
# Input an Image Here
image = image.open('Lab1\Lab data-20230926\map0.png').convert('L')
grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255 # binarize the image
grid_map[grid_map > 0.5] = 1 
grid_map[grid_map <= 0.5] = 0 # Invert colors to make 0 -> free and 1 -> occupied 
grid_map = (grid_map * -1) + 1 # Show grid map 

# Defin All Usefull Functions Here :
def is_onboard( position , map ):
    try:
        map[position] and (position[0] >= 0 and position[1] >= 0) #Also Assumes -ve Index as Out of boarder 
    except (ValueError, IndexError):
        return False
    else:
        return True   
def is_not_obstacle( position , map): # Checks a Position is obstacle or Not
    if(map[position] == 1):
        return False
    return True
def isValid( position , map): 
    if is_onboard(position , map ) and map[position] != 1 and  map[position] == 0   :
        return True
    return False
def getDistance(motion): # returns a distance based the connectivity we select 
    return 1 if conn_type=="2" else sqrt(motion[0]**2 + motion[1]**2)
def index_obstacles(map): # returns list of index in obstacle position
    rows , cols = map.shape
    list_obstacles = []
    for i in range(rows):
        for j in range(cols):
            if(map[i][j] == 1):
                list_obstacles.append((i,j))
    return list_obstacles
def getX(path):# returns the row part of two dimensional array 
    x=[]       # we use this to draw path in a map
    for p in path:
        i,j = p
        x.append(i) 
    return x
def getY(path):  # returns the colomun part of two dimensional array 
    y=[]         # we use this to draw path in a map
    for p in path:
        i,j = p
        y.append(j)
    return y
    
# Checking For Input Positions
#   1.Checking If Input Start Is Not Obstacle and In the Board
#   2.Checking If Input Goal is Not Obstacle and In The Board

if(not is_not_obstacle(start,grid_map)): raise Exception("Start Position is an Obstacle!")
if(not is_onboard(start,grid_map)): raise Exception("Start Position is Ouside The Board!")
if(not is_not_obstacle(goal,grid_map)): raise Exception("Goal Position is an Obstacle!")
if(not is_onboard(goal,grid_map)): raise Exception("Goal Position is Ouside The Board!") 



# Wave Front Algorithm 
def wavefront_planner(map):
    # wave_map = []
    wave_map = map.copy()
    wave_map[goal] = 2
    queue = [goal]
    i=0
    while queue: 

        new_queue = []
        for p in queue:
            for m in motions:   
                distance = getDistance(m)   
                new_position = (p[0] + m[0], p[1]+m[1])                 
                if isValid(new_position , wave_map):
                    wave_map[new_position] = wave_map[p] + getDistance(m)
                    new_queue.append(new_position)                 
            queue = new_queue
            i+=1
    return wave_map;

wave_front_map = wavefront_planner(grid_map)

# Attraction Function  and Normalization process
#     1. Replace the obstacles value by the maximum value in the Attraction function + 1 and 
#     2. then, normalize the function between 0 (for the goal) and 1 (for the obstacles). 
    
max_val = np.max(wave_front_map)
print(max_val)
attraction_fun = np.where( wave_front_map == 1 , max_val + 1 , wave_front_map.copy())
attraction_fun = np.where( attraction_fun == 2 , 0 ,attraction_fun ) # change the goal position to 0 value
attraction_fun = np.array( attraction_fun)/(max_val+1) #  divide by the max value 
# Path Finder Algorithm Here: returns the list path , distance and 
#    1. By applaying possible motion finds the best path 
#    2. if it is not reachable it returns a list upto the local minimum and boolean value find_goal= False
def find_path(start,goal,map):
    path = [start]   
    path_dist = 0
    find_goal = True # If we traped to local mini find_goal flag become false 
    mini_position = start  
    i = 0   
    while goal != path[-1]: 
        i = i +1 
        np.random.shuffle(motions) 
        for m in motions:
            distance = getDistance(m)
            current_position = path[-1]
            new_position = (current_position[0] + m[0], current_position[1] + m[1])       
            if(is_onboard(new_position,map) and is_not_obstacle(new_position,map) and map[new_position] < map[mini_position] ):
                mini_position = new_position

        if(mini_position == current_position): # if we visit a cell for the second time it is local min 
            find_goal = False                  # so we return the path upto local min and find_goal= False
            print("Local Minimum" , mini_position)
            return (path,path_dist,find_goal)
            # raise Exception("The Position" , current_position , "is Local Minimum ")
        else: 
            path.append(mini_position)
            path_dist += distance
    
    return (path,path_dist,find_goal)

      
path_attr_func,pathdis_attr_func,find_goal= find_path(start,goal,attraction_fun)  # path for attraction function call 

# Bursh Fire Algorithm Here:
def brushfire_planner(map):
    brushfire_map =map.copy()
    queue = index_obstacles(brushfire_map) # get all index of obstacles 
    value = 1
    i=0
    while queue: 
        value +=1
        new_queue = []
        for p in queue:
            for m in motions:       
                new_position = ( p[0] + m[0], p[1] + m[1])
                if isValid(new_position , brushfire_map):
                    brushfire_map[new_position] = value 
                    new_queue.append(new_position)
                        
            queue = new_queue
            i+=1

    return brushfire_map;

brushfire_map = brushfire_planner(grid_map)
# Normalized BurshFire map

repulsive_fun = np.where((brushfire_map<=Q) & (brushfire_map>1), 4*((1/brushfire_map)-(1/Q))**2, brushfire_map)
repulsive_fun = np.where( repulsive_fun >Q , 0, repulsive_fun)

## Potential Function 
potential_map = np.add(repulsive_fun , attraction_fun)
# potential_map = np.where(potential_map>1 ,1 , potential_map)
potential_map = np.divide(potential_map,2) # normalizing the potential function by dividing by its maximum value which is 2
path_potential_func,pathdis_pot_func,find_goal= find_path(start,goal,potential_map) # Potential function path function call


print("\nAttraction Function => Path Distance: {} , Number of Paths:{} ".format(pathdis_attr_func,len(path_attr_func)))
if(find_goal==False):
    print("Using Potential Function We traped to Local Mini at :{} After {} Paths \n".format(path_potential_func[-1],len(path_potential_func)))
else:
    print(" Potential Function => Path Distance: {} ,  Number of Paths:{} \n".format(pathdis_pot_func,len(path_potential_func)))

print("Attraction Function Path Is :\n {}\n".format(path_attr_func))
print("Potentail Function Path Is :\n  {}\n".format(path_potential_func))


plt.matshow(potential_map)
plt.colorbar()
# 
plt.plot(getY(path_potential_func),getX(path_potential_func),color='red')
plt.scatter(start[1], start[0], s=300, c='red', marker='.')
plt.scatter(goal[1], goal[0], s=300, c='red', marker='*')
plt.show()

# plt.matshow(brushfire_map)
# # plt.plot(getY(path_attr_func.copy()),getX(path_attr_func.copy()),color='red')
# plt.title('brushfire')
# plt.colorbar()
# plt.show()

# plt.matshow(repulsive_fun)
# # plt.plot(getY(path_attr_func.copy()),getX(path_attr_func.copy()),color='red')
# plt.title('Normalized Brushfire(Repulsive Fun)')
# plt.colorbar()
# plt.show()

# All Plotes Here 
# fig, axs = plt.subplots(2, 4 ,figsize=(15, 15))
# plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
# a= axs[0, 0].matshow(grid_map)
# axs[0, 0].set_title('Orginal')
# # fig.colorbar(a)
# axs[0, 1].matshow(wave_front_map)
# axs[0, 1].set_title('wave front')

# axs[0, 2].matshow(attraction_fun)
# axs[0, 2].set_title('Attraction Function')

# axs[0, 3].matshow(attraction_fun)
# axs[0, 3].plot(getY(path_attr_func.copy()),getX(path_attr_func.copy()),color='red')
# axs[0, 3].set_title('Path of Attraction Function!')

# axs[1, 0].set_title('Brushfire Function')
# axs[1, 0].matshow(brushfire_map)
# fig.colorbar(axs[1, 0].matshow(brushfire_map))
# axs[1, 1].set_title('Repulsive Function')
# axs[1, 1].matshow(repulsive_fun)

# axs[1, 2].set_title('Potential Function')
# axs[1, 2].matshow(potential_map)

# axs[1, 3].set_title('Path of Potential Function')
# axs[1, 3].matshow(potential_map)

# axs[1, 3].plot(getY(path_potential_func),getX(path_potential_func),color='red')
# axs[1,3].scatter(start[1], start[0], s=100, c='red', marker='.')
# axs[1,3].scatter(goal[1], goal[0], s=100, c='black', marker='.')


plt.show()