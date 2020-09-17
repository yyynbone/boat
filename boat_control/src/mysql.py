#!/home/hsm/miniconda3/bin/ python3
import pymysql
from copy import  deepcopy
import networkx as nx
import math
import numpy as np

def millerToXY(lon, lat):
    L = 6381372.0 * math.pi * 2.0 
    W = L  
    H = L / 2.0  
    mill = 2.3  
    x = float(lon) * math.pi / 180.0  
    y = float(lat) * math.pi / 180.0
    y = 1.25 * math.log(math.tan(0.25 * math.pi + 0.4 * y))  
   
    x = (W / 2) + (W / (2 * math.pi)) * x
    y = (H / 2) - (H / (2 * mill)) * y
    return int(round(x)), int(round(y))

def distance(id1,id2,dictionary):
    #print(id1,id2)
    #print("dic : ",dictionary)
    gps1=dictionary[str(id1)]
    gps2=dictionary[str(id2)]
    x1,y1= millerToXY(gps1[0],gps1[1])
    x2,y2 = millerToXY(gps2[0], gps2[1])
    return (x1-x2)**2+(y1-y2)**2

def get_graph():
    db = pymysql.connect("47.100.92.173","dev","Beimeng403", "waterway_map" )
    cursor = db.cursor()
    sql = "select * from node "
    cursor.execute(sql)
    data = cursor.fetchall()


    id_gps={}
    for x in data:
        X,Y=millerToXY(x[2],x[3])
        id_gps[str(x[0])] = [x[2],x[3],X,Y]


    sql = "select * from way_node "
    cursor.execute(sql)
    data = cursor.fetchall()

    path_node=[]
    for x in data:
        path_node.append([x[1],x[3],x[2]])   # way_num sort id
    path_node=sorted(path_node, key = lambda x:x[1])
    path_node=sorted(path_node, key = lambda x:x[0])

    G = nx.Graph()
    for i in range (len(path_node)):
        if i>0:
            if path_node[i-1][0]==path_node[i][0]:
                node_id1 = path_node[i - 1][2]
                node_id2 = path_node[i][2]
                G.add_weighted_edges_from([(node_id1,node_id2 , distance(node_id1,node_id2,id_gps))])
    # list_save = nx.to_dict_of_dicts(G)
    # way_nodes = nx.shortest_path(G, source=source1, target=target1, weight=True, method='dijkstra')
    # H = nx.from_dict_of_dicts(list_save)
    db.commit()
    db.close()
    return G,id_gps

