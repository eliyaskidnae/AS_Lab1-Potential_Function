# Input Data
# Select Algorithm( 4-point , 8-point , Eculidian)  Switch Case
# 
# import necessary 
import PIL.Image as image
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from math import sqrt


print("Please Select Connectivity Type ")
print("\n Enter 1 for 4-point Connectivity");
print(" Enter 2 for 8-point Connectivity");
print(" Enter 3 to use Euclidean distance ");

start = (10,10)
goal = (90,70)
Q = 5
motions = []
conn_type = 1 # default connectivivity type
conn_type = input("")

match conn_type:
    case '1': # 4-point connectivity
        motions = [(-1,0),(1,0),(0,-1),(0,1)]
    case '2': # 8-point connectivity
        motions = [(-1,0),(1,0),(0,-1),(0,1),(1,1),(-1,-1),(1,-1),(-1,1)]
    case '3': # Euclidean Distance
        motions = [(-1,0),(1,0),(0,-1),(0,1),(1,1),(-1,-1),(1,-1),(-1,1)]
    case _: # default 4-point connectivity 
        motions = [(-1,0),(1,0),(0,-1),(0,1)]
        
# Input 
image = image.open('Lab1\Lab data-20230926\map0.png').convert('L')
grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255 # binarize the image
grid_map[grid_map > 0.5] = 1 
grid_map[grid_map <= 0.5] = 0 # Invert colors to make 0 -> free and 1 -> occupied 
grid_map = (grid_map * -1) + 1 # Show grid map 

# Usefull Functions Here :
def is_onboard( position , arr ):
    try:
        arr[position] and (position[0] >= 0 and position[1] >= 0) #Also Assumes -ve Index as Out of boarder 
    except (ValueError, IndexError):
        return False
    else:
        return True   
def is_not_obstacle( position ,arr):
    if(arr[position] == 1):
        return False
    return True
def isValid( position , map):
    if is_onboard(position , map ) and map[position] != 1 and  map[position] == 0   :
        return True
    return False
def getDistance(motion):
    return 1 if conn_type==2 else sqrt(motion[0]**2 + motion[1]**2)
def index_obstacles(map):
    rows , cols = map.shape
    list_obstacles = []
    for i in range(rows):
        for j in range(cols):
            if(map[i][j] == 1):
                list_obstacles.append((i,j))
    return list_obstacles
def getX(path):
    x=[] 
    for p in path:
        i,j = p
        x.append(i) 
    return x
def getY(path):
    y=[] 
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
    value = 2 
    wave_map[goal] = value
    queue = [goal]
    i=0
    while queue: 
        # value +=1
        new_queue = []
        for p in queue:
            for m in motions:   
                distance = getDistance(m)
                value +=distance    
                new_position = (p[0] + m[0], p[1]+m[1])                 
                if isValid(new_position , wave_map):
                    wave_map[new_position] = value
                    new_queue.append(new_position)                 
            queue = new_queue
            i+=1
    return wave_map;

wave_front_map = wavefront_planner(grid_map)

# Attraction Function  and Normalization process
    # print(new_map)
    #  Replace the obstacles value by the maximum value in the Attraction function + 1 and 
    #  then, normalize the function between 0 (for the goal) and 1 (for the obstacles).
    
max_val = np.max(wave_front_map)
attraction_fun = np.where( wave_front_map == 1 , max_val + 1 , wave_front_map.copy())
attraction_fun = np.where( attraction_fun == 2 , 0 ,attraction_fun ) # change the goal to 0 
attraction_fun = np.array( attraction_fun)/(max_val+1)


# Path Finder Algorithm Here:
def find_path(start,goal,map):
    path = [start]   
    mini_position = start  
    i = 0   
    while goal != path[-1]: 
        i = i +1 
        np.random.shuffle(motions) 
        for m in motions:
            current_position = path[-1]
            new_position = (current_position[0] + m[0], current_position[1] + m[1])       
            if(is_onboard(new_position,map) and is_not_obstacle(new_position,map) and map[new_position] < map[mini_position] ):
                print(current_position , new_position)
                mini_position = new_position
                
        
        print(path)
        print('\n')
        print(mini_position)
        if(mini_position == current_position):
            print("Local Minimum")
            return path
            # raise Exception("The Position" , current_position , "is Local Minimum ")
        else: path.append(mini_position)
    print(path)
    return path


path_attr_func = find_path(start,goal,attraction_fun)
# path plotting helper here
# Bursh Fire Algorithm Here:
def bushfire_planner(map):
    bushfire_map =map.copy()
    queue = index_obstacles(bushfire_map) # get all index of obstacles 
    value = 1
    i=0
    while queue: 
        value +=1
        new_queue = []
        for p in queue:
            for m in motions:       
                new_position = ( p[0] + m[0], p[1] + m[1])
                if isValid(new_position , bushfire_map):
                    bushfire_map[new_position] = value 
                    new_queue.append(new_position)
                        
            queue = new_queue
            i+=1

    return bushfire_map;

bushfire_map = bushfire_planner(grid_map)
print("length " ,bushfire_map.max() )
# Normalized BurshFire map

normalized_bushfire_map = np.where((bushfire_map<=Q) & (bushfire_map>1), 4*((1/bushfire_map)-(1/Q))**2, bushfire_map)
normalized_bushfire_map = np.where( normalized_bushfire_map >Q , 0, normalized_bushfire_map)

# repulsive_fun = np.where((bushfire_map<=Q) & (bushfire_map>1), 4*(1/(bushfire_map)-(1/Q))*2, bushfire_map.copy())
# repulsive_fun = np.where( bushfire_map > Q , 0 , repulsive_fun)
repulsive_fun = normalized_bushfire_map

## Potential Function 
potential_map = np.add(repulsive_fun , attraction_fun)
# potential_map = np.where(potential_map>1 ,1 , potential_map)
potential_map = np.divide(potential_map,2) # normalizing the potential function by dividing by its maximum value which is 2
# print(potential_map[66,88])
# print("\n")
# print(potential_map[61:70,84:100])


path_potential_func = find_path(start,goal,potential_map)



fig, axs = plt.subplots(2, 4 ,figsize=(15, 15))
plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
a= axs[0, 0].matshow(grid_map)
axs[0, 0].set_title('Orginal')
# fig.colorbar(a)
axs[0, 1].matshow(wave_front_map)
axs[0, 1].set_title('wave front')

axs[0, 2].matshow(attraction_fun)
axs[0, 2].set_title('Attraction Function')

axs[0, 3].matshow(attraction_fun)
axs[0, 3].plot(getY(path_attr_func.copy()),getX(path_attr_func.copy()),color='red')
axs[0, 3].set_title('Path of Attraction Function!')

axs[1, 0].set_title('busfire Function')
axs[1, 0].matshow(bushfire_map)

axs[1, 1].set_title('Repulsive Function')
axs[1, 1].matshow(repulsive_fun)

axs[1, 2].set_title('Potential Function')
axs[1, 2].matshow(potential_map)

axs[1, 3].set_title('Path of Potential Function')
axs[1, 3].matshow(potential_map)
axs[1, 3].plot(getY(path_potential_func),getX(path_potential_func),color='red')
axs[1,3].scatter(start[1], start[0], s=100, c='red', marker='.')
axs[1,3].scatter(goal[1], goal[0], s=100, c='green', marker='.')


plt.show()