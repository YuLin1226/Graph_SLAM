import numpy as np

class PoseGraph():

    def __init__(self):
        '''
        Parameter:
        * Node : Pose nodes in graph
        * Edge : Edge in graph
        * H    : Information matrix
        * b    : Information vector

        * n_node : Number of nodes in graph     
        * n_edge : Number of edges in graph
        * pose   : Poses of all nodes
        
        '''
        pass

    def add_zero_constructor(self, node, edge):
        self.node = np.zeros(np.shape(node))
        self.edge = np.zeros(np.shape(edge))
        return

    def read_data(self, node_set, edge_set):
        '''
        node_set: array of node data :4xN
              [0]:  id (list)
              [1]:   x (list)
              [2]:   y (list)
              [3]: yaw (list)

        edge_set: array of edge data :11:N
              [0]: id_from (list)
              [1]:   id_to (list)
            [2~4]:    mean (list)
           [5~10]:    infm (list)
        '''

        self.node[0,:] = node_set[0,:]
        self.node[1,:] = node_set[1,:]
        self.node[2,:] = node_set[2,:]
        self.node[3,:] = node_set[3,:]



        self.edge[0,:] = edge_set[0,:]
        self.edge[1,:] = edge_set[1,:]
        self.edge[2,:] = [edge_set[2:4,:]]
        for i in range(len(edge_set)):

            self.edge[3,i] = [
                [edge_set[5,i], edge_set[6,i], edge_set[9,i]],
                [edge_set[6,i], edge_set[7,i], edge_set[10,i]],
                [edge_set[9,i], edge_set[10,i], edge_set[8,i]]
            ]
    
    def optimize(self, iteration):
        for i in iteration:



    def iterate(self):
        pass
    
    def linearize(self):
        pass

    def solve(self):
        pass

class PoseNode():
    
    def __init__(self, x, y, yaw, rt):
        '''
        
        id    : Id of this pose node
        pose  : Pose of this pose node
        
        x    : X coordinate
        y    : Y coordinate
        yaw  : Yaw angle
        rt   : Transformation local to global
        '''
        self.x   = x
        self.y   = y
        self.yaw = yaw
        self.rt  = rt

    def get_info(self, id, pose):
        '''
        資料還需做處理。
        '''
        
        self.id   = id
        self.pose = pose
        return




class PoseEdge():
    
    def __init__(self):
        '''
        id_from  : This is the viewing frame of this edge
        id_to    : This is the pose being observed from the viewing frame
        mean     : Predicted virtual measurement
        infm     : Information matrix of this edge
        '''
        pass

    def add_info(self, id_from, id_to, mean, infm):
        '''
        資料還需做處理。
        '''
        self.id_from = id_from
        self.id_to   = id_to
        self.mean    = mean
        self.infm    = infm

        return