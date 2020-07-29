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

        '''(Done)
        Create zeros constructor of node and edge.
        ------------------------------------------
        
        (5xn array)
        self.node = [
             id |     |  id >> index                        : int
              x |     |   x >> pose of x                    : float
              y | ... |   y >> pose of y                    : float
            yaw |     | yaw >> pose of yaw                  : float
             rt |     |  rt >> transformation  : 3x3 array  : float
        ]

        (4xn array)
        self.edge = [
            id_from |     | >> index                               : int
              id_to | ... | >> index                               : int
               mean |     | >> relative transformation : 3x1 array : float
               infm |     | >> covariance information  : 3x3 array : float
        ]
        '''
        self.node = np.zeros(np.shape(node))
        self.edge = np.zeros(np.shape(edge))
        return

    def read_data(self, node_set, edge_set):

        '''(Done)
        Read the node and edge data from the files.
        -------------------------------------------
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
        # put in node data.
        self.node[0,:] = node_set[0,:]
        self.node[1,:] = node_set[1,:]
        self.node[2,:] = node_set[2,:]
        self.node[3,:] = node_set[3,:]

        for i in range(len(node_set)):
            self.node[4,i] = np.array([
                [np.cos(node_set[3,i]), -np.sin(node_set[3,i]), node_set[1,i]],
                [np.sin(node_set[3,i]),  np.cos(node_set[3,i]), node_set[2,i]],
                [                    0,                      0,             1]
            ]) 

        # put in edge data
        self.edge[0,:] = edge_set[0,:]
        self.edge[1,:] = edge_set[1,:]

        for i in range(len(edge_set)):
            self.edge[2,i] = np.array([
                edge_set[2,i], edge_set[3,i], edge_set[4,i]
            ])
        
            self.edge[3,i] = np.array([
                [ edge_set[5,i], edge_set[6,i],  edge_set[9,i]  ],
                [ edge_set[6,i], edge_set[7,i],  edge_set[10,i] ],
                [ edge_set[9,i], edge_set[10,i], edge_set[8,i]  ]
            ])
    
    def optimize(self, num_iteration):

        '''(Done)
        Implement optimization to find a best solution for the graph.
        Optimization will stop when maximal iteration is reached.
        '''
        for i in num_iteration:
            print("No. %d iteration of optimization ..." %(i+1))
            self.iterate_graph_slam()


    def iterate_graph_slam(self):
        
        '''
        Iteration of pose graph optimization
        Details of the matrice below refer to paper "A Tutorial on Graph-Based SLAM."
        H : 3n x 3n matrix
        b : 3n x 1  matrix
        '''

        print("Iteration...")
        # Create new H and b matrices each time
        # obj.H = zeros(obj.n_node*3);   % 3n x 3n square matrix
        # obj.b = zeros(obj.n_node*3,1); % 3n x 1  column vector
        
        self.H = np.zeros( (len(self.node), len(self.node)) )
        self.b = np.zeros( (len(self.node),1) )

        print("Linearization...")
        self.linearize_err_fcn()

        print("Solve the linear system...")
        self.solve_lin_sys()
            
    def linearize_err_fcn(self):
        
        '''
        Linearize error functions and formulate a linear system
        '''
        
        for i_edge in range( len(self.edge) ):
            # No. i constraint
            ei = self.edge[:, i_edge]

            # i_node: id_from
            # j_node: id_to
            i_node  = ei[0]
            j_node  = ei[1]

            # T_z: Transformation Matrix
            T_z = self.v2t(ei[2])

            # omega: Convariance
            omega = ei[3]

            # v_i: pose of node i : x, y, yaw
            # v_j: pose of node j : x, y, yaw
            v_i = np.array([
                self.node[0, i_node], self.node[1, i_node], self.node[2, i_node]
            ])
            v_j = np.array([
                self.node[0, j_node], self.node[1, j_node], self.node[2, j_node]
            ])

            # Construct transformation from node to global frame
            T_i = self.v2t(v_i)
            T_j = self.v2t(v_i)

            R_i = T_i[0:2, 0:2]
            R_z = T_z[0:2, 0:2]

            # 1st order Derivative 
            si = np.sin(v_i[2])
            ci = np.cos(v_i[2])
            dR_i = np.array([
                [-si,  ci],
                [-ci, -si]
            ])
            dt_ij = v_j[0:2] - v_i[0:2]

            # Calculation of Jacobians
            A = np.hstack( np.dot(-R_z.transpose(), R_i.transpose()) , np.dot(np.dot(R_z.transpose(), dR_i), dt_ij) )
            A = np.vstack(A, np.array([0,0,-1]))

            B = np.hstack(np.dot(R_z.transpose(), R_i.transpose()), np.array([[0],[0]]))
            B = np.vstack(B, np.array([0,0,1]))

            # Calculation of error vector
            e = self.t2v( np.linalg.inv(T_z).dot( np.linalg.inv(T_i) ).dot(T_j) )
            
            # Formulation of updated data of H & b
            H_ii =  A.transpose().dot(omega).dot(A)
            H_ij =  A.transpose().dot(omega).dot(B)
            H_ji =  H_ij.transpose()
            H_jj =  B.transpose().dot(omega).dot(B)
            b_i  =  -A.transpose().dot(omega).dot(e)
            b_j  =  -B.transpose().dot(omega).dot(e)

            # Index of updated data 
            i_ind_start, i_ind_end = self.id2ind(i_node)
            j_ind_start, j_ind_end = self.id2ind(j_node)

            # Update H and b matrix
            self.H[i_ind_start : i_ind_end , i_ind_start : i_ind_end] = self.H[i_ind_start : i_ind_end , i_ind_start : i_ind_end] + H_ii
            self.H[i_ind_start : i_ind_end , j_ind_start : j_ind_end] = self.H[i_ind_start : i_ind_end , j_ind_start : j_ind_end] + H_ij
            self.H[j_ind_start : j_ind_end , i_ind_start : i_ind_end] = self.H[j_ind_start : j_ind_end , i_ind_start : i_ind_end] + H_ji
            self.H[j_ind_start : j_ind_end , j_ind_start : j_ind_end] = self.H[j_ind_start : j_ind_end , j_ind_start : j_ind_end] + H_jj
            
            self.b[i_ind_start : i_ind_end] = self.b[i_ind_start : i_ind_end] + b_i
            self.b[j_ind_start : j_ind_end] = self.b[j_ind_start : j_ind_end] + b_j


    def solve_lin_sys(self):
        pass


    def v2t(self, vector):
        
        '''(Done)
        vector to homogeneous transformation
        From local to global
        [              |
              Rotaion  | Translation
          _____________|____________
             0   |   0 |      1
        ]
        '''
        c = np.cos(vector[2])
        s = np.sin(vector[2])
        T = np.array([
            [c,  -s,  vector[0]],
            [s,   c,  vector[1]],
            [0,   0,          1]
        ])
        return T

    def id2ind(self, id):
        
        '''(Done)
        Converts id to indices in H and b
        '''
        ind_start = 3*id - 3
        ind_end   = 3*id
        return ind_start, ind_end

    def t2v(self, T):
        
        '''(Done)
        homogeneous transformation to vector
        '''
        v[0:1, 0] = T[0:1, 2]
        v[2, 0] = np.arctan2(A[1,0], A[0,0])
        return v






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