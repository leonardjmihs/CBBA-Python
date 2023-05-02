#!/usr/bin/env python3
import time
import json
import matplotlib.pyplot as plt
import pathmagic
from dataclasses import dataclass
from dacite import from_dict
import numpy as np
with pathmagic.context():
    from CBBA import CBBA
    from WorldInfo import WorldInfo
    import HelperLibrary as HelperLibrary
    from Agent import Agent
    from Task import Task
    from Obstacle import Obstacle
    from window import Window


if __name__ == "__main__":
    # a json configuration file
    config_file_name = "config_example_01.json"
    # Read the configuration from the json file
    json_file = open(config_file_name)
    config_data = json.load(json_file)

    # create a world, each list is [min, max] coordinates for x,y,z axis
    WorldInfoTest = WorldInfo([-2.0, 2.5], [-1.5, 5.5], [0.0, 20.0])

    # create a list of Agent(s) and Task(s)
    num_agents = 5
    num_tasks = 10
    num_obs = 3
    max_depth = num_tasks
    # AgentList, TaskList, obsList = HelperLibrary.create_agents_and_tasks_and_obstacles(num_agents, num_tasks, num_obs, WorldInfoTest, config_data)

    # save_agents = []
    # for agent in AgentList: 
    #     al_dict = agent.__dict__
    #     save_agents.append(al_dict)

    # save_tasks = []
    # for task in TaskList: 
    #     tas_dict = task.__dict__
    #     save_tasks.append(tas_dict)

    # save_obs = []
    # for obs in obsList: 
    #     obs_dict = obs.__dict__
    #     save_obs.append(obs_dict)
    # case_dict = {'AgentList':save_agents, 'TaskList':save_tasks, 'obsList':save_obs}
    # with open('case3', 'w') as fout:
    #     json.dump(case_dict, fout)
    AgentList = []
    TaskList = []
    obsList = []

    with open('case2', 'r') as fin:
        data = json.load(fin)
        for agent in data['AgentList']: 
            AgentList.append(from_dict(data_class=Agent, data=agent))
        for task in data['TaskList']: 
            TaskList.append(from_dict(data_class=Task, data=task))

        for obs in data['obsList']: 
            obsList.append(from_dict(data_class=Obstacle, data=obs))


    # create a CBBA solver
    CBBA_solver = CBBA(config_data)

    t_start = time.time()

    # path_list, times_list = CBBA_solver.solve(AgentList, TaskList, WorldInfoTest, max_depth, time_window_flag=False, obsList=obsList)
    # # solve
    if False:
        window = Window("CBBA-EGO")
        window.show(block=False)
        window.ax.set_ylim([-2.0, 2.5])
        window.ax.set_xlim([-1.5, 5.5])
        path_list, times_list = CBBA_solver.solve(AgentList, TaskList, WorldInfoTest, max_depth, time_window_flag=True, obsList=obsList)
        current_index = [0 for _ in range(num_agents)]
        ts =0
        while True:
        # path_list, times_list = CBBA_solver.solve(AgentList, TaskList, WorldInfoTest, max_depth, time_window_flag=False, obsList=obsList)
        # for i in range(0,20):
            ts += 1
            
            offset = (CBBA_solver.WorldInfo.limit_x[1]-CBBA_solver.WorldInfo.limit_x[0]) / 100
            window.ax.cla()
            for m in range(CBBA_solver.num_tasks):
                # track task is red
                if CBBA_solver.TaskList[m].task_type == 0:
                    color_str = 'red'
                # rescue task is blue
                else:
                    color_str = 'blue'
                window.ax.scatter(CBBA_solver.TaskList[m].x, CBBA_solver.TaskList[m].y, marker='x', color=color_str)
                window.ax.text(CBBA_solver.TaskList[m].x+offset, CBBA_solver.TaskList[m].y+offset, "T"+str(m))

            # plot agents
            for n in range(CBBA_solver.num_agents):
            # for n in range(1):
                # quad  red
                if CBBA_solver.AgentList[n].agent_type == 0:
                    color_str = 'red'
                # car agent is blue
                else:
                    color_str = 'blue'
                window.ax.scatter(CBBA_solver.AgentList[n].x, CBBA_solver.AgentList[n].y, marker='o', color=color_str)
                window.ax.text(CBBA_solver.AgentList[n].x+offset, CBBA_solver.AgentList[n].y+offset, "A"+str(n))
                if CBBA_solver.path_list[n]:
                    Task_next, m = CBBA_solver.lookup_task(CBBA_solver.path_list[n][current_index[n]])
                    dir_x = (Task_next.x - AgentList[n].x)
                    dir_y = (Task_next.y - AgentList[n].y)
                    dir_theta = np.arctan2(dir_y,dir_x )
                    print(f"Agent {[n]}: ({AgentList[n].x}, {AgentList[n].y}) Task_next: {Task_next.task_id}, ({Task_next.x}, {Task_next.y}) Theta: {dir_theta*180/np.pi}")
                    AgentList[n].x += np.cos(dir_theta)*AgentList[n].nom_velocity*0.025
                    AgentList[n].y += np.sin(dir_theta)*AgentList[n].nom_velocity*0.025

                    if np.linalg.norm([AgentList[n].x-Task_next.x, AgentList[n].y-Task_next.y]) < 0.1:
                        AgentList[n].x = Task_next.x
                        AgentList[n].y = Task_next.y
                        # break
                        if current_index[n] < len(path_list[n])-1:
                            current_index[n]+=1
                        continue
                        # TaskList[CBBA_solver.path_list[n][0]]
                        # if CBBA_solver.path_list[n][m] > -1:
                        #     Task_next = CBBA_solver.lookup_task(CBBA_solver.path_list[n][m])
                        #     window.ax.plot([Task_prev.x, Task_next.x], [Task_prev.y, Task_next.y], linewidth=2, color=color_str)
                            #Task_prev = Task(**Task_next.__dict__)

                    Task_prev, ind = CBBA_solver.lookup_task(CBBA_solver.path_list[n][current_index[n]])
                    window.ax.plot([CBBA_solver.AgentList[n].x, Task_prev.x], [CBBA_solver.AgentList[n].y, Task_prev.y],
                            linewidth=2, color=color_str)
                    for m in range(1, len(CBBA_solver.path_list[n])):
                        if CBBA_solver.path_list[n][m] > -1:
                            Task_next, ind = CBBA_solver.lookup_task(CBBA_solver.path_list[n][m])  
                            window.ax.plot([Task_prev.x, Task_next.x], [Task_prev.y, Task_next.y], linewidth=2, color=color_str)
                            Task_prev = Task(**Task_next.__dict__)
            window.ax.set_xlim([-2.0, 2.5])
            window.ax.set_ylim([-1.5, 5.5])
            window.fig.canvas.draw()
            window.mypause(0.001)          
    else:
        path_list, times_list = CBBA_solver.solve(AgentList, TaskList, WorldInfoTest, max_depth, time_window_flag=True, obsList=obsList)



    
    t_end = time.time()
    t_used = t_end - t_start
    print("Time used [sec]: ", t_used)

    # the output is CBBA_solver.path_list or path_list
    print("bundle_list")
    print(CBBA_solver.bundle_list)
    print("path_list")
    print(path_list)
    print("times_list")
    print(times_list)
    print("winners_list")
    print(CBBA_solver.winners_list)
    print("scores_list")
    print(CBBA_solver.scores_list)

    print("bid_list")
    print(CBBA_solver.bid_list)
    print("winner_bid_list")
    print(CBBA_solver.winner_bid_list)

    # plot
    # CBBA_solver.plot_assignment()
    # plt.figure()
    CBBA_solver.plot_assignment_without_timewindow()
    for t in CBBA_solver.TaskList:
        print(t)
    plt.show()
