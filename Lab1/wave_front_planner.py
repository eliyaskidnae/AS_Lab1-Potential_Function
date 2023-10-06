


def wavefront_planner_connect_4(map,goal):
    wave_map =map
    wave_map[goal] = 2
    motions = [(-1,0),(1,0),(0,-1),(0,1),(1,1),(-1,-1),(1,-1),(-1,1)] #  [left, right, up, down] (1,1),(-1,-1),(1,-1),(-1,1),
    value = 2 
    wave_map[goal] = value
    queue = [goal]
    i=0
    print("Initial Input Data")
    print(wave_map)
    while queue: 
        # value +=1
        new_queue = []
        for p in queue:
            for m in motions:   
                distance = sqrt(m[0]**2 + m[1]**2)
                value +=distance    
                new_position = (p[0] + m[0], p[1]+m[1])
                    # print(new_position , isValid(new_position,wave_map))
                if isValid(new_position , wave_map):
                    # print(distance)
                    wave_map[new_position] = value
                    new_queue.append(new_position)
                        
            queue = new_queue
            print(" Iteration ",i)
            print(new_queue)
            i+=1
 
    
    return wave_map;